# Rapidly Exploring Random Trees (RRT)
# Created by Ashwin Vinoo
# Date: 3/9/2019

# importing the necessary modules
from scipy import spatial
import numpy as np
import random
import math
import time
import warnings


# Euclidean distance is used as the heuristic
def euclidean_distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


# Checks if there is a collision between starting points and ending points
def check_hit(map_matrix, start_coordinate, end_coordinate):

    # In case start coordinate is equal to end coordinate
    if start_coordinate == end_coordinate:
        # If these points overlap an obstacle then return as a hit
        if map_matrix[start_coordinate[0], start_coordinate[1]] == 0:
            return True
        else:
            return False

    # Obtaining the change in x and the change in y
    dx = end_coordinate[1] - start_coordinate[1]
    dy = end_coordinate[0] - start_coordinate[0]
    # This works assuming that a 1x1 cell can only be obstacle or free
    cell_coverage = math.ceil(max(abs(dx), abs(dy)))
    # Getting the scan segment lengths for both axis
    dx /= cell_coverage
    dy /= cell_coverage
    # x and y are the variables we will be incrementing
    x = start_coordinate[1]
    y = start_coordinate[0]
    # Iterating across the line between start and goal coordinates by deltas
    for i in range(cell_coverage+1):
        # Checking if any intermediate coordinate is beyond the map or covers an obstacle
        if(x < 0 or y < 0 or x >= map_matrix.shape[1] or y >= map_matrix.shape[0] or
           map_matrix[math.floor(y), math.floor(x)] == 0 or
           map_matrix[math.floor(y), math.ceil(x)] == 0 or
           map_matrix[math.ceil(y), math.floor(x)] == 0 or
           map_matrix[math.ceil(y), math.ceil(x)] == 0):
            # We return true to indicate that there is a hit with an obstacle or the line lies outside bounds
            return True
        # For the last iteration, we will directly load the end coordinates
        if i == cell_coverage-1:
            # Loading the end coordinates
            x = end_coordinate[1]
            y = end_coordinate[0]
        else:
            # Increment x and y by the deltas
            x = x + dx
            y = y + dy
    # We return false if we can't find a single case in which there is a collision
    return False


# Finds the path via RRT algorithm and returns the path, distance to goal and the computation time
def find_path(map_matrix, start, goal, rrt_growth_limit, terminal_goal_distance,
              rrt_goal_epsilon=0.1, rrt_maximum_nodes=10000):

    # We measure the time at start
    start_time = time.time()
    # We initalize the node list with the start coordinates
    node_list = [start]
    # We initalize the node parent list that tells child nodes where it connects (key-value pair)
    node_parent = {}
    # We create a variable to hold the current tree size (number of nodes)
    node_count = 0
    # We initialize the goal honing distance to zero
    goal_honing_distance = 0

    # We run the while loop until we are at a terminal distance from the goal
    while True:
        # An epsilon percentage chance that we will grow the tree towards goal rather than a random coordinate
        if rrt_goal_epsilon > random.random():
            # The goal is the coordinate towards which we pull the tree
            random_coordinate = goal
        else:
            # Generate a random coordinate towards which we will pull the tree
            random_coordinate = [int((map_matrix.shape[0]-1)*random.random()),
                                 int((map_matrix.shape[1]-1)*random.random())]

        # Calculating the distance between the nearest node and the random coordinate and its index in the node list
        distance, index = spatial.KDTree(node_list).query(random_coordinate)
        # Obtains the node nearest to the randomly sampled coordinate
        nearest_node = node_list[index]
        # Getting the y position of the node to be added
        y_node = nearest_node[0] + rrt_growth_limit/distance*(random_coordinate[0]-nearest_node[0])
        # Getting the x position of the node to be added
        x_node = nearest_node[1] + rrt_growth_limit/distance*(random_coordinate[1]-nearest_node[1])
        # Limiting the Y_node to stay within the map boundary in case it crosses over
        y_node = min(max(y_node, 0), map_matrix.shape[0]-1)
        # Limiting the Y_node to stay within the map boundary in case it crosses over
        x_node = min(max(x_node, 0), map_matrix.shape[1]-1)
        # Representing both as a single coordinate. We will try to attempt to extend the tree to this point
        coordinate_node = (y_node, x_node)

        # Checking to see if there are obstacles between the two coordinates
        if not check_hit(map_matrix, nearest_node, coordinate_node):
            # We increment the node count by one
            node_count += 1
            # We add the new node to the node list
            node_list.append(coordinate_node)
            # We only take the first index found in the rare case that two or more matches appear
            node_parent[coordinate_node] = nearest_node
            # The distance between the current node and the goal node
            goal_honing_distance = euclidean_distance(coordinate_node, goal)
            # if the node added is within terminal distance from goal, we are done
            if euclidean_distance(coordinate_node, goal) < terminal_goal_distance:
                # We increment the node count by one
                node_count += 1
                # We add the goal node to the node list
                node_list.append(goal)
                # We mark the current node as parent of goal node
                node_parent[goal] = coordinate_node
                # terminates the while loop
                break
            elif node_count >= rrt_maximum_nodes:
                # Warning if goal wasn't reached within node limit
                warnings.warn('RRT failed to find a solution within the node limit')
                # We measure the time to perform the path planning
                end_time = time.time()
                # Returns empty lists to show that the solution was not found
                return [], [], end_time-start_time, [], None

    # We create a variable to contain the RRT nodes in path
    rrt_path_nodes_count = 1
    # A list to store the nodes in the path
    nodes_in_path = [goal]
    # This variable helps in the parent node tracing process
    node_latest = goal
    # While loop terminates when we retrace back to starting node
    while node_latest != start:
        # Gets the 'from' node via the the 'to' node
        node_latest = node_parent[node_latest]
        # We obtain the coordinates of the node that is the parent
        nodes_in_path.append(node_latest)
        # We increment the nodes in path count by 1
        rrt_path_nodes_count += 1

    # The path length in the (number of node in path - 2) * rrt growth limit + goal honing distance
    rrt_path_length = (rrt_path_nodes_count-2)*rrt_growth_limit + goal_honing_distance
    # We measure the time to perform the path planning
    end_time = time.time()
    # This list will hold the pair of coordinates that the rrt branches across
    nodes_in_branches = []

    # We iterate through all the elements of the node list other than start node
    for node in node_list[1:]:
        # The parent node of the current node
        parent_node = node_parent[node]
        # We append the node and the node it was branched from to the list
        nodes_in_branches.append([(parent_node[0], parent_node[1]), (node[0], node[1])])

    # Returns the nodes in the path and the path length
    return nodes_in_path[:-1], rrt_path_length, end_time-start_time, nodes_in_branches, node_count
