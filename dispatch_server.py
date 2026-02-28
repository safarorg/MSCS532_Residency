import csv
import heapq
from drone import DeliveryException
from drone import Drone
from order import Order


class DispatchServer(object):

  def __init__(self, delivery_zones):
    self.delivery_zones = delivery_zones
    self.trips = []
    self.unpackaged_orders = []
    self.delivery_drone = Drone(delivery_zones)
    self.payload_test_drone = Drone(delivery_zones)

  def _priority_key(self, order):
    """
    Composite key for the optimal-path min-heap: (priority_score, timestamp, delivery_zone, order_id).
    Lower priority_score = higher urgency. delivery_zone then order_id as tiebreakers;
    order_id guarantees a total order and deterministic heap behavior.
    """
    priority_score = (0 if order.is_perishable() else 2) + (0 if order.is_subscriber() else 1)
    return (priority_score, order.get_timestamp(), order.get_delivery_zone(), order.get_order_id())

  def package_trips(self, most_optimal):
    """
    Uses build_trip (or build_most_optimal_trip) to organize the
    unpackaged orders into a list of packaged trips.
    Simple and optimal paths are independent: simple uses unpackaged_orders
    directly; optimal uses whatever order list is passed to build_most_optimal_trip.
    """
    if most_optimal:
      self._package_trips_optimal()
    else:
      self._package_trips_simple()

  def _package_trips_simple(self):
    """Simple path: FIFO by timestamp, zone-based trips. Uses only unpackaged_orders."""
    self.unpackaged_orders.sort(key=lambda o: o.get_timestamp())

    while self.unpackaged_orders:
      earliest_order = self.unpackaged_orders[0]
      trip = self.build_trip(earliest_order.get_delivery_zone())
      if not trip:
        break
      for order in trip:
        self.unpackaged_orders.remove(order)
      self.trips.append(trip)

  def _package_trips_optimal(self):
    """
    Optimal path: min-heap priority queue. Orders dequeued by (priority_score,
    timestamp, delivery_zone, order_id). heapq.heappush/heappop maintain the heap invariant.
    """
    # Load orders into min-heap with composite key (priority_score, timestamp, delivery_zone, order_id)
    heap = []
    for order in self.unpackaged_orders:
      key = self._priority_key(order)
      heapq.heappush(heap, (key, order))

    while heap:
      # Pop all from heap; they come out in priority order.
      items = [heapq.heappop(heap) for _ in range(len(heap))]
      orders_only = [o for _, o in items]

      trip = self.build_most_optimal_trip(orders_only)
      if not trip:
        # No trip could be built; put all back and stop.
        for key, o in items:
          heapq.heappush(heap, (key, o))
        break

      # Remove trip orders from unpackaged_orders; push back the rest to heap.
      for order in trip:
        self.unpackaged_orders.remove(order)
      trip_set = set(trip)
      for key, o in items:
        if o not in trip_set:
          heapq.heappush(heap, (key, o))

      self.trips.append(trip)

  def deliver_orders(self):
    """Uses the delivery_drone to deliver all of the packaged trips."""
    while self.trips:
      trip = self.trips[0]
      for order in trip:
        self.delivery_drone.add_order(order, 
          len(self.delivery_drone.get_orders()))
      try:
        self.trips.pop(0)
        self.delivery_drone.deliver_orders()  # Release the drone!
      except DeliveryException as e:
        print(e)
        return
      self.delivery_drone.recharge()


  def build_trip(self, zone):
    """
    Returns a Trip containing as many of the oldest unpackaged Orders in the
    zone as possible without exceeding the drone's range. Once an order exceeds
    the drone's range, any remaining capacity should be filled with other orders
    destined for this same zone from heaviest to lightest (regardless of when
    the order was placed).
    """
    zone_orders = [o for o in self.unpackaged_orders if o.get_delivery_zone() == zone]

    self.payload_test_drone.remove_all_orders()
    trip = []

    # Add oldest orders first until one exceeds range
    for order in zone_orders:
      best_pos = self.payload_test_drone.find_best_order_position(order)
      if best_pos >= 0:
        self.payload_test_drone.add_order(order, best_pos)
        trip.append(order)
      else:
        break

    # Fill remaining capacity with other zone orders, heaviest to lightest
    remaining = [o for o in zone_orders if o not in trip]
    remaining.sort(key=lambda o: o.get_weight(), reverse=True)
    for order in remaining:
      best_pos = self.payload_test_drone.find_best_order_position(order)
      if best_pos >= 0:
        self.payload_test_drone.add_order(order, best_pos)
        trip.append(order)

    self.payload_test_drone.remove_all_orders()
    return trip

  def build_most_optimal_trip(self, orders):
    """
    Returns the most optimal Trip from the given orders list, addressing:
    - Perishable goods at front of queue
    - Subscriber priority
    - Same customer orders together (when they fit)
    - Never mix fragile and hazardous in same trip
    Does not mutate orders; caller is responsible for removing trip orders.
    """
    if not orders:
      return []

    # Sort: perishable first, subscriber second, timestamp third (front of queue)
    sorted_orders = sorted(orders,
        key=lambda o: (not o.is_perishable(), not o.is_subscriber(), o.get_timestamp()))

    first = sorted_orders[0]

    # Trip type: never mix fragile and hazardous
    if first.is_fragile():
      trip_type = 'no_hazardous'
    elif first.is_hazardous():
      trip_type = 'no_fragile'
    else:
      trip_type = 'any'

    def can_add(order):
      if trip_type == 'no_hazardous':
        return not order.is_hazardous()
      elif trip_type == 'no_fragile':
        return not order.is_fragile()
      return True

    self.payload_test_drone.remove_all_orders()
    trip = []

    # Add all same-customer orders first (unless they can't all fit)
    customer_id = first.get_user_id()
    customer_orders = [o for o in sorted_orders if o.get_user_id() == customer_id and can_add(o)]
    for order in customer_orders:
      best_pos = self.payload_test_drone.find_best_order_position(order)
      if best_pos >= 0:
        self.payload_test_drone.add_order(order, best_pos)
        trip.append(order)
      else:
        break

    # Fill remaining capacity: other orders by priority, then heaviest to lightest
    remaining = [o for o in sorted_orders if o not in trip and can_add(o)]
    remaining.sort(key=lambda o: (not o.is_perishable(), not o.is_subscriber(),
        o.get_timestamp(), -o.get_weight()))
    for order in remaining:
      best_pos = self.payload_test_drone.find_best_order_position(order)
      if best_pos >= 0:
        self.payload_test_drone.add_order(order, best_pos)
        trip.append(order)

    # Return orders in optimal delivery order (as positioned by find_best_order_position)
    trip = self.payload_test_drone.get_orders().copy()
    self.payload_test_drone.remove_all_orders()
    return trip

  def load_orders(self, csv_file_name):
    print('Loading order info from ' + csv_file_name)
    with open(csv_file_name, 'r') as csvfile:
      reader = csv.reader(csvfile, delimiter=',')
      for parsed_line in reader:
        self.add_order(int(parsed_line[0]),
                       int(parsed_line[1]),
                       int(parsed_line[2]),
                       int(parsed_line[3]),
                       int(parsed_line[4]),
                       parsed_line[5] == 'TRUE',
                       parsed_line[6] == 'TRUE', 
                       parsed_line[7] == 'TRUE',
                       parsed_line[8] == 'TRUE')

  def add_order(self, order_id=0, timestamp=0, zone=0,
                weight=0, user_id=0, subscriber=False, fragile=False,
                hazardous=False, perishable=False):

    order = Order(order_id, timestamp, zone, weight, user_id, subscriber,
                    fragile, hazardous, perishable)

    self.unpackaged_orders.append(order)
