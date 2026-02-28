"""
Returns the distance in kilometers between the centers of the given origin
and destination zones.
"""
import csv

class DeliveryZones(object):

  def __init__(self):
    self.distance_matrix = [[]]

  def add_distance(self, origin, destination, distance):
    self.distance_matrix[origin][destination] = distance

  def load_matrix(self, csv_file_name):
    print("Loading distance info from " + csv_file_name)
    try:
      with open(csv_file_name, "r") as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        first_line = next(reader)[0]
        num_cells = int(first_line)
        self.distance_matrix = [[0] * num_cells for _ in range(num_cells)]

        origin = 0
        for line in reader:
          if line:
            for destination in range(len(line)):
              self.add_distance(origin, destination, int(line[destination]))
          origin += 1
    except IOError as e:
      print(f"Error importing order from {csv_file_name}\n {e}")

  def distance_between(self, origin_zone, destination_zone):
    """
    Returns the distance in kilometers between the centers of the given origin
    and destination zones.
    """
    matrix_size = len(self.distance_matrix)
    if (0 <= origin_zone < matrix_size) and (0 <= destination_zone < matrix_size):
      return self.distance_matrix[origin_zone][destination_zone]
