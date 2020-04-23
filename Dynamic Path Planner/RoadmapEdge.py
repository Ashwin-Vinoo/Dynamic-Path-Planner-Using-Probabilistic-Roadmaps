# RoadmapEdge Class holds details on the edges such as the nodes and other edges it is connected with
# Created by Ashwin Vinoo
# Date: 3/9/2019

# We import the necessary modules
from rrt import check_hit, euclidean_distance


# Class Integrated_PRM contains the functionality needed to implement the Integrated PRM path planning algorithm
class RoadmapEdge(object):

    # The class constructor takes in coordinates of the two vertices
    def __init__(self, vertex_1, vertex_2, edge_length=None):
        # We store the first coordinate
        self.vertex_1 = vertex_1
        # We store the second coordinate
        self.vertex_2 = vertex_2
        # We store the edge length
        self.edge_length = edge_length
        # We check if the edge length is None
        if edge_length is None:
            # We store the euclidean distance between the vertices as the edge length
            self.edge_length = euclidean_distance(vertex_1, vertex_2)
        # We use this variable to mark if an edge is over an dynamic obstacle
        self.dynamic_obstacle_overlap = False

    # This function computes whether the edge overlaps a dynamic obstacle or not
    def update_edge_blockage(self, dynamic_obstacle_list, world_map):
        # We start by marking that there is no overlap
        self.dynamic_obstacle_overlap = False
        # We iterate through the dynamic obstacles in the obstacle list
        for dynamic_obstacle in dynamic_obstacle_list:
            # The following conditions are to check if there is a possibility of the edge overlapping the obstacle
            if not ((self.vertex_1[0] < dynamic_obstacle.min_y and self.vertex_2[0] < dynamic_obstacle.min_y) or
                    (self.vertex_1[0] > dynamic_obstacle.max_y and self.vertex_2[0] > dynamic_obstacle.max_y) or
                    (self.vertex_1[1] < dynamic_obstacle.min_x and self.vertex_2[1] < dynamic_obstacle.min_x) or
                    (self.vertex_1[1] > dynamic_obstacle.max_x and self.vertex_2[1] > dynamic_obstacle.max_x)):
                # We check for any collisions with the dynamic obstacle
                if check_hit(world_map, self.vertex_1, self.vertex_2):
                    # We mark that there has been an overlap
                    self.dynamic_obstacle_overlap = True
                    # We break the for loop as we have verified a collision
                    break



