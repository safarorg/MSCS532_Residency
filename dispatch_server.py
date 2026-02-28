import csv
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
        trip = self.build_trip(earliest_order.get_delivery_zone())

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

  def build_most_optimal_trip(self):
    """
    Returns the most optimal Trip, addressing:
    - Perishable goods at front of queue
    - Subscriber priority
    - Same customer orders together (when they fit)
    - Never mix fragile and hazardous in same trip
    """
    if not self.unpackaged_orders:
      return []

    # Sort: perishable first, subscriber second, timestamp third (front of queue)
    sorted_orders = sorted(self.unpackaged_orders,
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
