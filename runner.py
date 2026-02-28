from delivery_zones import DeliveryZones
from dispatch_server import DispatchServer
from drone import Drone
from order import Order

class Runner:

  def run(self):
    print("Initializing Server...")
    delivery_zones = DeliveryZones()
    dispatch_server = DispatchServer(delivery_zones)
    delivery_zones.load_matrix("distances.csv")
    dispatch_server.load_orders("deliveries.csv")

    print("\n*** Test drone ***\n")
    test_drone = Drone(delivery_zones)
    test_order_zone = 11
    good_order_weight = 10
    too_big_order_weight = 50000
    test_good_order = Order(0, 0, test_order_zone, good_order_weight, 0, False, False, False, False)
    test_bad_order = Order(0, 0, test_order_zone, too_big_order_weight, 0, False, False, False, False)

    good_order_position = test_drone.find_best_order_position(test_good_order)
    print("Accommodates good order: ", good_order_position >= 0)

    bad_order_position = test_drone.find_best_order_position(test_bad_order)
    print("Accommodates bad order: ", bad_order_position >= 0)

    print("\n*** Basic Delivery ***\n")
    dispatch_server.package_trips(False)
    dispatch_server.deliver_orders()  # Release the drones!

    print("\n*** Optimized Delivery ***\n")
    dispatch_server.load_orders("deliveries.csv")
    dispatch_server.package_trips(True)
    dispatch_server.deliver_orders()  # Release the drones!

    #  Feel free to put any println statements below for testing and debugging

if __name__ == "__main__":
    Runner().run()
