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

  def package_trips(self, most_optimal):
    """
    Uses build_trip (or build_most_optimal_trip) to organize the
    unpackaged orders into a list of packaged trips.
    """
    # Sort orders by submitted time
    self.unpackaged_orders.sort(key=lambda o: o.get_timestamp())

    while self.unpackaged_orders:
      trip = []

      # Build a trip
      if most_optimal:
        trip = self.build_most_optimal_trip()
      else:
        earliest_order = self.unpackaged_orders[0]
        trip = self.build_trip(
          earliest_order.get_delivery_zone())

      # Exit the loop if no orders are added to the trip
      if not trip:
        break

      for order in trip:
        self.unpackaged_orders.remove(order)

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
    # unpackaged_orders is already sorted by timestamp (from package_trips)
    zone_orders = [o for o in self.unpackaged_orders
                    if o.get_delivery_zone() == zone]

    trip = []
    remaining_for_phase2 = []

    # Phase 1: Add oldest orders while battery permits; stop at first failure
    for i, order in enumerate(zone_orders):
      pos = len(self.payload_test_drone.get_orders())
      result = self.payload_test_drone.simulate_trip_with_added_order(
          order, pos)
      if result >= 0:                          # FIX: -1 is falsy but means failure
        self.payload_test_drone.add_order(
            order, pos)  # FIX: accumulate state
        trip.append(order)
      else:
        # this order and all after
        remaining_for_phase2 = zone_orders[i:]
        break

    # Phase 2: Fill remaining capacity heaviest-to-lightest
    remaining_for_phase2.sort(
        key=lambda o: o.get_weight(), reverse=True)
    for order in remaining_for_phase2:
      # FIX: use current count
      pos = len(self.payload_test_drone.get_orders())
      result = self.payload_test_drone.simulate_trip_with_added_order(
        order, pos)
      if result >= 0:
        self.payload_test_drone.add_order(order, pos)
        trip.append(order)

    self.payload_test_drone.remove_all_orders()         # FIX: always reset
    return trip

  def build_most_optimal_trip(self):
    """
    Returns the most optimal Trip using a Priority Queue + Customer Grouping
    algorithm.

    Algorithm:
      1. Sort all unpackaged orders by urgency:
          perishable > subscriber > oldest timestamp.
      2. Group orders by user_id (hash map) so each customer's orders travel
          together wherever possible.
      3. Push each user group onto a min-heap keyed by its top-order priority.
      4. Pop groups in priority order; enforce the fragile/hazardous
          co-existence constraint; insert each fitting order at its optimal
          position (find_best_order_position) to minimize battery consumption.

    Time complexity: O(n log n) overall (dominated by the initial sort).
    Space complexity: O(n) for the group map and heap.
    """
    # Step 1: Sort by urgency (perishable first, then subscriber, then oldest)
    sorted_orders = sorted(
        self.unpackaged_orders,
        key=lambda o: (0 if o.is_perishable() else 1,
                        0 if o.is_subscriber() else 1,
                        o.get_timestamp())
    )

    # Step 2: Group by user_id using a dict (hash map)
    user_groups = {}
    for order in sorted_orders:
        uid = order.get_user_id()
        if uid not in user_groups:
            user_groups[uid] = []
        user_groups[uid].append(order)

    # Step 3: Push each group onto the min-heap
    # Heap key: (perishable_flag, subscriber_flag, timestamp, uid)
    # uid is appended as an integer tiebreaker to avoid comparing lists
    heap = []
    for uid, group in user_groups.items():
        # highest-priority order in this group (from Step 1 sort)
        top = group[0]
        score = (0 if top.is_perishable() else 1,
                  0 if top.is_subscriber() else 1,
                  top.get_timestamp(),
                  uid)
        heapq.heappush(heap, score)

    # Step 4: Greedily build one trip by processing groups in priority order
    trip = []
    trip_has_fragile = False
    trip_has_hazardous = False

    while heap:
      entry = heapq.heappop(heap)
      uid = entry[3]
      group = user_groups[uid]

      # Enforce fragile/hazardous constraint: fragile and hazardous goods
      # must not travel on the same trip
      group_fragile = any(o.is_fragile() for o in group)
      group_hazardous = any(o.is_hazardous() for o in group)
      if (trip_has_fragile and group_hazardous) or \
              (trip_has_hazardous and group_fragile):
          continue  # defer this group to a future trip

    # Try to add each order from this customer at its optimal position
    for order in group:
      best_pos = self.payload_test_drone.find_best_order_position(
          order)
      if best_pos >= 0:
        self.payload_test_drone.add_order(order, best_pos)
        trip_has_fragile = trip_has_fragile or order.is_fragile()
        trip_has_hazardous = trip_has_hazardous or order.is_hazardous()

    # Return orders in the optimized delivery sequence that find_best_order_position
    # computed, not in the processing order — the delivery drone must fly this sequence
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
