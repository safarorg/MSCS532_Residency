class Order(object):

  def __init__(self, order_id, order_timestamp, delivery_zone, weight, user_id,
               subscriber, fragile, hazardous, perishable):
    self.order_id = order_id
    self.timestamp = order_timestamp
    self.delivery_zone = delivery_zone
    self.weight = weight
    self.user_id = user_id
    self.subscriber = subscriber
    self.fragile = fragile
    self.hazardous = hazardous
    self.perishable = perishable

  def get_order_id(self):
    return self.order_id

  def get_timestamp(self):
    return self.timestamp

  def get_delivery_zone(self):
    return self.delivery_zone

  def get_weight(self):
    return self.weight

  def get_user_id(self):
    return self.user_id

  def is_subscriber(self):
    return self.subscriber

  def is_fragile(self):
    return self.fragile

  def is_hazardous(self):
    return self.hazardous

  def is_perishable(self):
    return self.perishable

  def __str__(self):
    return (f"Order {self.order_id} at {self.timestamp} to {self.delivery_zone}"
            f" ({self.weight}g)")
