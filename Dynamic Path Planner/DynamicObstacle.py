# DynamicObstacle Class holds details on the dynamic obstacles that move around in the static map
# Created by Ashwin Vinoo
# Date: 3/9/2019


# Class Integrated_PRM contains the functionality needed to implement the Integrated PRM path planning algorithm
class DynamicObstacle(object):

    # The class constructor takes in coordinates of the two vertices
    def __init__(self, coordinate_list):
        # The obstacle coordinate list holds the coordinates that the dynamic obstacle occupies currently
        self.obstacle_coordinate_list = coordinate_list
        # We initalize the minimum values for the x and y coordinates as
        self.min_y = None
        self.min_x = None
        self.max_y = None
        self.max_x = None
        # In case the length of the coordinate list is greater than zero
        if len(coordinate_list) > 0:
            # We call the function to identify the minimums and maximums along both axes
            self.find_ranges()

    # This function helps add a coordinate to the coordinate list
    def add_coordinate_to_obstacle(self, coordinate):
        # We add the coordinate to the obstacle coordinate list
        self.obstacle_coordinate_list.append(coordinate)

    # This function computes the minimums and maximums along both axes
    def find_ranges(self):
        # The y-coordinates in order
        y_coordinates = [item[0] for item in self.obstacle_coordinate_list]
        # The x-coordinates in order
        x_coordinates = [item[1] for item in self.obstacle_coordinate_list]
        # The minimum of the y-coordinates is obtained
        self.min_y = min(y_coordinates)
        # The maximum of the y-coordinates is obtained
        self.max_y = max(y_coordinates)
        # The minimum of the x-coordinates is obtained
        self.min_x = min(x_coordinates)
        # The maximum of the x-coordinates is obtained
        self.max_x = max(x_coordinates)

