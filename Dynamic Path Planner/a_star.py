# Author: Christian Careaga (christian.careaga7@gmail.com) Modified by Ashwin Vinoo
# A* Path finding in Python (3.6)
# Date: 3/9/2019

# importing the necessary modules
import numpy as np
import heapq as hq
import time
import math


# Euclidean distance is used as the heuristic
def euclidean_distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


# Finds the shortest path via A* algorithm
def find_path(map_matrix, start, goal):

    # We measure the time at start
    start_time = time.time()
    # Creates an unordered empty set
    close_set = set()
    # declares an empty object (key-value pairs)
    came_from = {}
    # The g-score of the start positioning (key-value pairs)
    g_score = {start: 0}
    # The f-score of the start positioning (key-value pairs)
    f_score = {start: euclidean_distance(start, goal)}
    # The open heap in which the coordinates to be expanded
    open_heap = []
    # Push the starting coordinates and its f-score
    hq.heappush(open_heap, (f_score[start], start))
    # The eight neighbour positions to consider (relative positioning)
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    # The nodes we have expanded
    expanded_nodes = []

    # While the open heap is not empty
    while open_heap:

        # Obtains the current coordinate to expand from the heap and its f_score
        current_coordinate_f_score, current_coordinate = hq.heappop(open_heap)
        # We add the coordinate to be expanded to the expanded nodes list
        expanded_nodes.append(current_coordinate)

        # If the current coordinate is the goal
        if current_coordinate == goal:
            # Create a list to store the path
            path_data = []
            # Looping through the came from section
            while current_coordinate in came_from:
                # Adding the current coordinate
                path_data.append(current_coordinate)
                # Getting the next coordinate in the list
                current_coordinate = came_from[current_coordinate]
            # We measure the time to perform the path planning
            end_time = time.time()
            # Returns the path data (from start to goal), path length, A* computation time and list of expanded nodes
            return path_data[:-1], current_coordinate_f_score, end_time-start_time, expanded_nodes

        # Adding the current coordinates to the close set
        close_set.add(current_coordinate)

        # Iterating through the possible neighbours
        for i, j in neighbors:

            # Getting the neighbour coordinate
            neighbor = (current_coordinate[0] + i, current_coordinate[1] + j)
            # Calculating the tentative g-score assuming we moved to this point from the current coordinate expanded
            tentative_g_score = g_score[current_coordinate] + euclidean_distance(current_coordinate, neighbor)

            # Checking if the y coordinate is within the allowed range
            if 0 <= neighbor[0] < map_matrix.shape[0]:
                # Checking if the x coordinate is within the allowed range
                if 0 <= neighbor[1] < map_matrix.shape[1]:
                    # Checks if the neighbor is an obstacle
                    if map_matrix[neighbor[0], neighbor[1]] == 0:
                        # stop further evaluation as neighbor is an obstacle
                        continue
                else:
                    # stop further evaluation as map_matrix bound x walls
                    continue
            else:
                # stop further evaluation as map_matrix bound y walls
                continue

            # If the neighbour is in closed set and its tentative g-score is greater than that calculated earlier
            if neighbor in close_set and tentative_g_score >= g_score.get(neighbor, 0):
                # stop further evaluation as the tentative g-score is greater than that via a previous route
                continue
            # If the tentative score is less or the neighbor is not in the open heap
            elif tentative_g_score < g_score.get(neighbor, 0) or neighbor not in [i[1]for i in open_heap]:
                # We specify that we reached the neighbor from the current coordinate
                came_from[neighbor] = current_coordinate
                # We update the g-score of the neighbor
                g_score[neighbor] = tentative_g_score
                # We update the f-score of the neighbor
                f_score[neighbor] = tentative_g_score + euclidean_distance(neighbor, goal)
                # We push the neighbor and its f-score to the open heap
                hq.heappush(open_heap, (f_score[neighbor], neighbor))

    # We measure the time to perform the path planning
    end_time = time.time()
    # We return an empty array to show that there isn't a path, infinity and run time if there isn't a path to the goal
    return [], float('Inf'), end_time-start_time, []
