import time
import traceback

from delivery_zones import DeliveryZones


class Drone(object):

  def __init__(self, delivery_zones):
    self.delivery_zones = delivery_zones
    self.battery_charge = 1
    self.orders = []

  def get_orders(self):
    """Returns the list of orders that this drone has committed to deliver."""
    return self.orders

  def add_order(self, order, position):
    """
    Adds the given order to the drone's list of orders at a given position.
    """
    self.orders.insert(position, order)

  def remove_all_orders(self):
    """Removes all orders from the drone."""
    self.orders = []

  def deliver_orders(self):
    """
    Deliver all of the orders attached to the drone. If the drone's battery dies
    during the trip, it will throw a DeliveryException.
    """
    self.battery_charge = self.run_trip(self.orders, False)
    if self.battery_charge < 0:
      raise DeliveryException("Drone battery died during trip!")

  def run_trip(self, orders, simulated_trip):
    """
    Delivers a list of orders attached to the drone, returning the battery
    charge level remaining on the drone after delivery. Returns -1 if the
    drone's battery dies during the trip, or on its way back to the
    warehouse.
    
    NOTE: If simulated_trip is true, this method simulates a trip,
    rather than actually sending the drone out. This can be useful for
    determining whether a drone will be able to complete a trip, given a
    particular list of orders.
    """
    previous_destination = 0
    payload_weight = 0
    battery_charge = 0
    for order in orders:
      payload_weight += order.get_weight()

    if simulated_trip:
      # We'll be simulating the trip, so we can assume a fully charged drone
      battery_charge = 1
    else: 
      # Actually deliver the orders, using the drone's current charge
      battery_charge = self.battery_charge
      print('Headed out for delivery:')

    delivered_order_count = 0
    for order in orders:
      if not simulated_trip:
        print(f'  Delivering Order #{order.get_order_id()}' +
              f' to zone {order.get_delivery_zone()}...')

      # Update drone battery, payload weight, and previous desination after
      # delivering the order
      battery_charge -= self.get_percent_battery_required(payload_weight,
        self.delivery_zones.distance_between(previous_destination,
          order.get_delivery_zone()))
      payload_weight -= order.get_weight()
      previous_destination = order.get_delivery_zone()

      # Exit the loop if the battery dies during the trip
      if battery_charge < 0:
        return -1
      elif not simulated_trip:
        delivered_order_count += 1
        print('    Complete!')

    # If this is a real trip, remove delivered orders from the list
    if not simulated_trip:
      for _ in range(delivered_order_count):
        orders.pop(0)
    
    battery_charge -= self.get_percent_battery_required(0,
      self.delivery_zones.distance_between(previous_destination, 0))

    if battery_charge < 0:
      return -1
    return battery_charge

  def simulate_trip_with_added_order(self, new_order, position):
    """
    Returns the resulting battery charge if the drone were to deliver all
    of the orders already in its order list, plus the given new_order (which
    would be added to the order list at the given position).
    """

  def find_best_order_position(self, new_order):
    """
    Returns the index of the best position to add an order to the drone's
    existing order list. The best position is the one where the drone returns
    to the warehouse with the greatest remaining battery charge. If there are
    no positions where the order can be added without the drone failing its
    trip, returns -1.
    """

  def recharge(self):
    """Sets the battery to full (1)."""
    self.battery_charge = 1

  def get_percent_battery_required(self, weight, distance):
    """
    Given a payload weight in grams and a distance in kilometers, returns
    the percent of drone range this flight would consume.

    A value of 1 indicates that the drone's entire battery is consumed.
    """
    empty_overhead = 512
    consumption_factor = 36739
    return distance * (weight + empty_overhead) / consumption_factor


class DeliveryException(Exception):
  pass
