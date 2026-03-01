import csv
import heapq
from drone import DeliveryException
from drone import Drone
from order import Order


_KEY_EXTRACTORS = {
    'user_id':        lambda o: o.get_user_id(),
    'delivery_zone':  lambda o: o.get_delivery_zone(),
    'timestamp':      lambda o: o.get_timestamp(),
    'priority_score': lambda o: (0 if o.is_perishable() else 2) + (0 if o.is_subscriber() else 1),
}


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
  
  def build_composite_key(self, order, sort_keys):
    """
    Composite key for the optimal-path min-heap built dynamically from sort_keys.
    Always appends order_id as the deterministic final tiebreaker.
    """
    key = tuple(_KEY_EXTRACTORS[k](order) for k in sort_keys)
    return key + (order.get_order_id(),)
  
  def package_trips(self):
    """
    Organizes unpackaged orders into trips using the priority queue (min-heap).
    Orders are dequeued by (priority_score, timestamp, delivery_zone, order_id).
    """
    self._package_trips_optimal()

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

  def build_trip(self, orders):
    """
    Build one trip from orders, enforcing only battery and weight constraints.
    Orders are processed in the sequence given; no fragile/hazardous/perishable logic.
    Skips any order that would exceed the weight limit or exhaust the battery.
    """
    trip = []
    current_weight = 0
    for order in orders:
      if current_weight + order.get_weight() > self.payload_test_drone.weight_limit:
        continue
      best_pos = self.payload_test_drone.find_best_order_position(order)
      if best_pos >= 0:
        self.payload_test_drone.add_order(order, best_pos)
        trip.append(order)
        current_weight += order.get_weight()
    trip = list(self.payload_test_drone.get_orders())
    self.payload_test_drone.remove_all_orders()
    return trip

  def schedule_orders(self, sort_keys):
    """
    Sorts unpackaged orders by sort_keys and greedily builds trips using
    build_trip (battery + weight constraints only).
    
    Sort all unpackaged orders once by sort_keys, then greedily fill trips
    using build_trip until no orders remain or none can fit.
    """
    orders = sorted(self.unpackaged_orders,
                    key=lambda o: self.build_composite_key(o, sort_keys))
    while orders:
      trip = self.build_trip(orders)
      if not trip:
        break  # remaining orders all oversized or battery-impossible; stop
      trip_set = set(id(o) for o in trip)
      orders = [o for o in orders if id(o) not in trip_set]
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

  def build_most_optimal_trip(self, orders):
    """
    Returns the most optimal Trip from the given orders list, addressing:
    - Perishable goods at front of queue
    - Subscriber priority
    - Same customer orders together (when they fit)
    - Never mix fragile and hazardous in same trip
    Does not mutate orders; caller is responsible for removing trip orders.
    Expects orders to already be in priority order (e.g. from the min-heap).
    """
    if not orders:
      return []

    # Already in priority order from heap
    first = orders[0]

    trip = []
    trip_has_fragile = False
    trip_has_hazardous = False

    # Phase 1: Add all same-customer orders first (when they satisfy constraints)
    customer_id = first.get_user_id()
    customer_orders = [o for o in orders if o.get_user_id() == customer_id]
    for order in customer_orders:
      if order.is_fragile() and trip_has_hazardous:
        continue
      if order.is_hazardous() and trip_has_fragile:
        continue
      best_pos = self.payload_test_drone.find_best_order_position(order)
      if best_pos >= 0:
        self.payload_test_drone.add_order(order, best_pos)
        trip.append(order)
        trip_has_fragile = trip_has_fragile or order.is_fragile()
        trip_has_hazardous = trip_has_hazardous or order.is_hazardous()

    # Phase 2: Fill remaining capacity, heaviest first
    already_added = set(id(o) for o in trip)
    remaining = [o for o in orders if id(o) not in already_added]
    remaining.sort(key=lambda o: -o.get_weight())
    for order in remaining:
      if order.is_fragile() and trip_has_hazardous:
        continue
      if order.is_hazardous() and trip_has_fragile:
        continue
      best_pos = self.payload_test_drone.find_best_order_position(order)
      if best_pos >= 0:
        self.payload_test_drone.add_order(order, best_pos)
        trip.append(order)
        trip_has_fragile = trip_has_fragile or order.is_fragile()
        trip_has_hazardous = trip_has_hazardous or order.is_hazardous()

    # Return orders in optimal delivery order (as positioned by find_best_order_position)
    trip = list(self.payload_test_drone.get_orders())
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
