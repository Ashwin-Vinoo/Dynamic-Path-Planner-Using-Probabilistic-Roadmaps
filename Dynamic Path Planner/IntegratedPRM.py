# Integrated PRM Technique
# Created by Ashwin Vinoo
# Date: 3/9/2019

# importing the necessary modules
from sklearn.neighbors import NearestNeighbors
from RoadmapEdge import RoadmapEdge
from rrt import check_hit
import numpy as np
import random
import copy
import heapq
import time


# Class Integrated_PRM contains the functionality needed to implement the Integrated PRM path planning algorithm
class IntegratedPRM(object):

    # mode can be 'density' or 'count' that applies to node_value and node_neighbors is the K-nearest neighbors for them
    def __init__(self, world_map, mode='density', node_value=10, node_neighbors=10, max_neighbor_distance=0.5):

        # ----------- world map settings and initializations -----------

        # Initializes the world map with that provided
        self.world_map = world_map
        # Obtains the number of rows in the world map
        self.world_map_rows = world_map.shape[0]
        # Obtains the number of columns in the world map
        self.world_map_columns = world_map.shape[1]

        # Initialize the number of world map nodes to be zero
        self.roadmap_nodes = 0
        # Check whether the number of roadmap nodes is 'auto' and not a fixed number
        if mode == 'density':
            # We obtain the number of roadmap nodes
            self.roadmap_nodes = int(np.count_nonzero(world_map)/(node_value**2))
            # We also change the max_neighbor_distance from density form to actual distance
            max_neighbor_distance = max_neighbor_distance * (self.world_map_rows + self.world_map_columns)/2
        else:
            # We directly use the number of roadmap nodes provided by the user
            self.roadmap_nodes = node_value

        # ----------- Adding nodes to the world map  -----------

        # We initialize the node list to be empty
        self.roadmap_node_list = []
        # We create a node dictionary that maps nodes to nodes and the distances between them
        self.roadmap_nodes_and_edge_dict = {}
        # We iterate through a while loop until we have randomly placed in all required nodes
        while len(self.roadmap_node_list) < self.roadmap_nodes:
            # We obtain a random row number
            random_row = random.randint(0, self.world_map_rows-1)
            # We obtain a random column number
            random_column = random.randint(0, self.world_map_columns-1)
            # Check if there is an static obstacle in world map at that point and if the coordinate is already listed
            if world_map[random_row, random_column] > 0 and (random_row, random_column) not in self.roadmap_node_list:
                # The roadmap node list is appended with the random coordinate
                self.roadmap_node_list.append((random_row, random_column))
                # We set the roadmap node to node dictionary at that coordinate to an empty list
                self.roadmap_nodes_and_edge_dict[(random_row, random_column)] = []

        # ----------- Creating edges using K nearest neighbors algorithm  -----------

        # We create an instance of the K nearest neighbors algorithm and fit it to handle the node list
        knn_algorithm = NearestNeighbors(n_neighbors=node_neighbors+1,
                                         algorithm='ball_tree').fit(self.roadmap_node_list)
        # We obtain the distances and indices to the K nearest neighbors of each node
        distances, indices = knn_algorithm.kneighbors(self.roadmap_node_list)
        # The list containing the edges of the roadmap
        self.roadmap_edge_list = []
        # Create a dictionary to map nodes that become part of an edge as true
        roadmap_node_list_dict = {}
        # Iterating through the rows of the indices where row number is index of current node in node list
        for i in range(len(self.roadmap_node_list)):
            # We obtain the indices of the nodes that link to the node at index i in the roadmap node list
            node_indices = indices[i][1:]
            # We iterate through indices of the nearest nodes associated with the current node
            for j in range(len(node_indices)):
                # We only consider indices greater than that of the current node to ensure repeats don't occur
                if (node_indices[j] > i and distances[i][j+1] <= max_neighbor_distance and
                        # We check if distance constraints are met and if adjacent nodes collide
                        not check_hit(world_map, self.roadmap_node_list[i], self.roadmap_node_list[node_indices[j]])):

                    # We create the new roadmap edge object
                    edge_object = RoadmapEdge(self.roadmap_node_list[i], self.roadmap_node_list[node_indices[j]],
                                              distances[i][j+1])
                    # We append the created roadmap edge object to the roadmap list
                    self.roadmap_edge_list.append(edge_object)

                    # We update the node to node dictionary for the first coordinate
                    self.roadmap_nodes_and_edge_dict[
                        self.roadmap_node_list[i]].append([self.roadmap_node_list[node_indices[j]], edge_object])
                    # We update the node to node dictionary for the second coordinate
                    self.roadmap_nodes_and_edge_dict[
                        self.roadmap_node_list[node_indices[j]]].append([self.roadmap_node_list[i], edge_object])

                    # We mark these nodes as true in the dictionary
                    roadmap_node_list_dict[self.roadmap_node_list[i]] = True
                    roadmap_node_list_dict[self.roadmap_node_list[node_indices[j]]] = True

        # ----------- Eliminating all nodes that do not have any connection to an edge -----------

        # Create a list to hold nodes that have a connection to an edge
        roadmap_node_list_edged = []
        # Iterating through the nodes in the node list
        for node in self.roadmap_node_list:
            # We check if that node has been marked as true in the dictionary
            if roadmap_node_list_dict.get(node, False):
                # Append the node onto the edged node list
                roadmap_node_list_edged.append(node)
        # We copy the edged nodes into the roadmap node list
        self.roadmap_node_list = roadmap_node_list_edged

    # This function checks whether the edges of the roadmap are blocked or not
    def update_edge_list_for_blockage(self, dynamic_obstacle_list, world_map):
        # We iterate through the edges of the roadmap
        for edge in self.roadmap_edge_list:
            # We update the edge status
            edge.update_edge_blockage(dynamic_obstacle_list, world_map)

    # We use this function to obtain the path from start to goal
    def find_path(self, map_matrix, dynamic_obstacle_list, start, goal):

        # We measure the time at start
        start_time = time.time()
        # We update the edges which are blocked by dynamic obstacles
        self.update_edge_list_for_blockage(dynamic_obstacle_list, map_matrix)

        # ----------- Obtaining the compatible node closest to the start -----------

        # We initialize the node to which we will connect the start node to as None
        start_node_in_roadmap = None
        start_node_distance = None
        # Iterating through the different number of neighbors possible
        for i in range(len(self.roadmap_node_list)):
            # We create an instance of the K Nearest neighbors algorithm that detects the closest 'i' neighbors
            knn_algorithm = NearestNeighbors(n_neighbors=i+1, algorithm='ball_tree').fit(self.roadmap_node_list)
            # We obtain the distances and indices to the K nearest neighbors to the start node
            distances, indices = knn_algorithm.kneighbors([start])
            # We check if there is no collision between the start location and the 'i'th closest node
            if not check_hit(map_matrix, start, self.roadmap_node_list[indices[0][i]]):
                # We obtain the point in roadmap to which the start node connects towards
                start_node_in_roadmap = self.roadmap_node_list[indices[0][i]]
                # We also store the distance between the start node and that in roadmap
                start_node_distance = distances[0][i]
                # We break the execution of the for loop as we found the required node
                break

        # ----------- Obtaining the compatible node closest to the goal -----------

        # We initialize the node to which we will connect the start node to as None
        goal_node_in_roadmap = None
        goal_node_distance = None
        # Iterating through the different number of neighbors possible
        for i in range(len(self.roadmap_node_list)):
            # We create an instance of the K Nearest neighbors algorithm that detects the closest 'i' neighbors
            knn_algorithm = NearestNeighbors(n_neighbors=i+1, algorithm='ball_tree').fit(self.roadmap_node_list)
            # We obtain the distances and indices to the K nearest neighbors of the goal node
            distances, indices = knn_algorithm.kneighbors([goal])
            # We check if there is no collision between the goal location and the 'i'th closest node
            if not check_hit(map_matrix, goal, self.roadmap_node_list[indices[0][i]]):
                # We obtain the point in roadmap to which the goal node connects towards
                goal_node_in_roadmap = self.roadmap_node_list[indices[0][i]]
                # We also store the distance between the goal node and that in roadmap
                goal_node_distance = distances[0][i]
                # We break the execution of the for loop as we found the required node
                break

        # ----------- Dijkstra's Algorithm -----------

        # We create an open list to indicate the node we want to expand via Dijkstra's algorithm
        open_list = list(self.roadmap_node_list.copy())
        # This dictionary marks the location of the coordinate from which we arrived at that point
        came_from = {start_node_in_roadmap: start, goal: goal_node_in_roadmap}
        # We create a list which will be used as a min heap
        node_heap = []
        # We initalize the heap with the starting node in the roadmap
        heapq.heappush(node_heap, (start_node_distance, start_node_in_roadmap))
        # We create an dictionary to hold the current values at every node
        node_cumulative_value = {}
        # We iterate through the nodes in the open list
        for node in self.roadmap_node_list:
            # We store the default cumulative values as infinite
            node_cumulative_value[node] = float('Inf')
        # We mark that the cumulative value of the starting node
        node_cumulative_value[start_node_in_roadmap] = start_node_distance

        # We mark that the goal has not been reached
        goal_reached = False
        # We initialize the path length as zero
        path_length = 0
        # We iterate through the while loop until we removed the goal_node_in_roadmap from the open list
        while True:
            # We check whether the node heap is empty
            if not node_heap:
                # We break the execution
                break
            # we pop the current node details from the heap
            current_node_details = heapq.heappop(node_heap)
            # We obtain the current node from the above
            current_node = current_node_details[1]
            # We check if the node we have popped is the goal:
            if current_node == goal_node_in_roadmap:
                # The path length is the cumulative value till the goal node in roadmap plus distance to goal
                path_length = current_node_details[0] + goal_node_distance
                # We indicate that the goal has been reached
                goal_reached = True
                # We break the execution
                break
            # We can't proceed if the current_node is not in the open list
            if current_node in open_list:
                # We remove the current node from the open list
                open_list.remove(current_node)
                # We obtain the cumulative value till the current node
                current_node_cumulative_value = current_node_details[0]
                # We obtain the list of nodes to which the current node is connected to
                connection_list = self.roadmap_nodes_and_edge_dict[current_node]
                # We iterate through the elements in the connection list
                for connect_details in connection_list:
                    # We obtain the connect node from the connect_details
                    connect_node = connect_details[0]
                    # We obtain the edge status
                    edge_blocked = connect_details[1].dynamic_obstacle_overlap
                    # we obtain the edge length of the connection
                    connect_length = connect_details[1].edge_length
                    # We obtain the cumulative path length till this connect node
                    connect_node_cumulative_value = current_node_cumulative_value + connect_length
                    # We check if the coordinate loaded is also in the open list and edge is not blocked
                    if (connect_node in open_list and not edge_blocked and
                            connect_node_cumulative_value < node_cumulative_value[connect_node]):
                        # We update the cumulative value at that point
                        node_cumulative_value[connect_node] = connect_node_cumulative_value
                        # We update the came from dictionary as this is the best path so far for the connect node
                        came_from[connect_node] = current_node
                        # We also add the connect node to the heap
                        heapq.heappush(node_heap, (connect_node_cumulative_value, connect_node))

        # We check if we have reached the goal
        if goal_reached:
            # The current node we a re tracing is the goal
            current_node = goal
            # We create a list onto which we can add the path and initialize it with the goal coordinate
            prm_path = [current_node]
            while current_node != start:
                # We obtain the node the current node came from. This is the new current node
                current_node = came_from[current_node]
                # We add this to the path
                prm_path.append(current_node)
            # We measure the time to perform the path planning
            end_time = time.time()
            # We return the path details
            return prm_path[::-1], path_length, end_time-start_time, self.roadmap_edge_list
        else:
            # We measure the time to perform the path planning
            end_time = time.time()
            # We failed to find a path so return
            return [], [], end_time-start_time, self.roadmap_edge_list
