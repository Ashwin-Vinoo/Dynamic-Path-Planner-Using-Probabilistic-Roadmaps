# Path Planner Comparison
# Created by Ashwin Vinoo
# Date 3/9/2019

# importing user defined modules
from IntegratedPRM import IntegratedPRM
from DynamicObstacle import DynamicObstacle
import a_star
import rrt
import line_plotter

# importing all the necessary modules
from PIL import Image
import numpy as np
import matplotlib.pyplot as plot
import time
import math
import random
import os

# Closes all pre-existing figures
plot.close("all")
# Obtains the path to the current working directory
directory_main = os.getcwd()
# The directory from which we may load maps
directory_maps = directory_main + '\Maps\\'
# The directory from which we may load dynamic obstacles
directory_obstacles = directory_main + '\obstacles\\'

# --------------------------------- Hyper parameters ---------------------------------
# Setting the random seed to ensure results don't change too much
random.seed(1)
# The list of maps to load
maps_to_load = 'map_4'
# The coordinate to start from
start_coordinate = (0, 0)
# The coordinate to end at
end_coordinate = (100, 199)
# This flag indicates how many dynamic obstacles we should load
obstacle_load_count = 1
# The list of dynamic obstacles to load
obstacles_to_load = ['obstacle_1', 'obstacle_2', 'obstacle_6']
# The coordinates over which the dynamic obstacle will be placed and scale into [(y1, x1), (y2, x2)]
obstacles_loaded_coordinates = [[(80, 50), (100, 100)], [(350, 300), (450, 400)], [(300, 80), (410, 180)]]
# ---------- PRM and RRT Parameters -----------
# The PRM node density
prm_node_density = 12
# The PRM node neighbors
prm_node_neighbors = 10
# The RRT growth limit
rrt_growth_limit = 10
# The RRT distance required to identify goal
rrt_goal_distance = 10
# ------------------------------------------------------------------------------------


# If this file is the main one called for execution
if __name__ == "__main__":

    # Reads in the bitmap world images (obstacles are zeros and free space are ones)
    world_map = Image.open(directory_maps + maps_to_load + '.bmp')
    # Converts the image into a numpy array
    world_map = np.abs(np.array(world_map))*255
    # Creates an RGB Version of the world map
    world_map_rgb = line_plotter.concat_channels(world_map, world_map, world_map)

    # ---------- Initializing the integrated PRM algorithm ----------
    # We record the start time
    start_time = time.time()
    # We create an object of the integrated PRM algorithm
    integrated_prm = IntegratedPRM(world_map, mode='density',
                                   node_value=prm_node_density, node_neighbors=prm_node_neighbors)
    # We print the computation time needed for setting up the probabilistic roadmap
    print('Time needed to initialize integrated PRM roadmap: ' + format(time.time()-start_time, '.2f') + ' seconds')

    # ---------- Loading Obstacles ----------
    # We create a list to store the dynamic obstacles
    dynamic_obstacle_list = []
    # We iterate through the obstacles to be loaded
    for i in range(obstacle_load_count):
        # We create an dynamic obstacle object
        dynamic_obstacle = DynamicObstacle([])
        # Reads in the bitmap images of the obstacles
        obstacle_image = Image.open(directory_obstacles + obstacles_to_load[i] + '.bmp')
        # Converts the obstacle image into a numpy array
        obstacle_image = np.abs(np.array(obstacle_image)-1)
        # The coordinates between which the obstacle should be scaled to fit
        obstacle_coordinates = obstacles_loaded_coordinates[i]
        # Initializing the step sizes in y-axis and x-axis to 1
        y_step = 1
        x_step = 1
        # In case the y coordinate of the second point is less than the first point
        if obstacle_coordinates[0][0] > obstacle_coordinates[1][0]:
            y_step = -1
        # In case the x coordinate of the second point is less than the first point
        if obstacle_coordinates[0][1] > obstacle_coordinates[1][1]:
            x_step = -1
        # The y-axis displacement
        dy = abs(obstacle_coordinates[0][0] - obstacle_coordinates[1][0])
        # The x-axis displacement
        dx = abs(obstacle_coordinates[0][1] - obstacle_coordinates[1][1])
        # Iterating through the y axis of the area on the world map to fit the obstacle
        for y in range(obstacle_coordinates[0][0], obstacle_coordinates[1][0] + y_step, y_step):
            # Iterating through the x axis of the area on the world map to fit the obstacle
            for x in range(obstacle_coordinates[0][1], obstacle_coordinates[1][1] + x_step, x_step):
                # The corresponding y coordinate in the obstacle image can be calculated as follows
                obstacle_y = (obstacle_image.shape[0]-1)/dy * abs(y - obstacle_coordinates[0][0])
                # The corresponding x coordinate in the obstacle image can be calculated as follows
                obstacle_x = (obstacle_image.shape[1]-1)/dx * abs(x - obstacle_coordinates[0][1])
                # We check whether there is an obstacle at this location in the obstacle image
                if (obstacle_image[math.floor(obstacle_y), math.floor(obstacle_x)] == 0 or
                        obstacle_image[math.floor(obstacle_y), math.ceil(obstacle_x)] == 0 or
                        obstacle_image[math.ceil(obstacle_y), math.floor(obstacle_x)] == 0 or
                        obstacle_image[math.ceil(obstacle_y), math.ceil(obstacle_x)] == 0):
                    # We mark that point in the world map as an obstacle
                    world_map[y, x] = 0
                    # We mark the dynamic obstacles as grey in the rgb world map
                    world_map_rgb[y, x, :] = [100, 100, 100]
                    # We add the coordinate to the dynamic obstacle object
                    dynamic_obstacle.add_coordinate_to_obstacle((y, x))
        # We update the ranges of the x and y axes for the dynamic obstacle
        dynamic_obstacle.find_ranges()
        # We add the dynamic obstacle object to the list
        dynamic_obstacle_list.append(dynamic_obstacle)

    # ---------- Plot Layout ----------
    # Specify that we require subplots along two rows and two columns
    figure, axis = plot.subplots(1, 3)
    # Specifying the primary plot title
    figure.suptitle('Results for map: ' + maps_to_load + ' with ' + repr(obstacle_load_count) + ' obstacles introduced')

    # ---------- A* Algorithm ----------
    # Uses the A_star algorithm to find the shortest route to the goal
    path, path_length, computation_time, coordinates_expanded = a_star.find_path(world_map, start_coordinate,
                                                                                 end_coordinate)
    # Adds lines to the plot
    line_plotter.plot_lines(axis[0], path, 'red')
    # We mark orange for all the coordinates expanded by the A* algorithm
    map_rgb_temp = line_plotter.color_marker(world_map_rgb, coordinates_expanded, [255, 165, 0])
    # Plotting the image with a grayscale colormap
    axis[0].imshow(map_rgb_temp, cmap='gray')
    # Specifying the plot title
    axis[0].set_title('A* run time: ' + format(computation_time, '.2f') +
                      ' seconds, path length: ' + format(path_length, '.2f'))
    # Specifying the label for the x-axis
    axis[0].set_xlabel('Map Columns')
    # Specifying the label for the y-axis
    axis[0].set_ylabel('Map Rows')
    # We print the results on terminal
    print('A* - The path length is: ' + format(path_length, '.2f') + ', the computation time is:' +
          format(computation_time, '.2f') + ' and the number of coordinates expanded is: ' +
          repr(len(coordinates_expanded)))

    # ---------- RRT Algorithm ----------
    # Uses the RRT algorithm to find a path to the destination
    path, path_length, computation_time, branch_set, node_count = rrt.find_path(world_map, start_coordinate,
                                                                                end_coordinate, rrt_growth_limit,
                                                                                rrt_goal_distance)
    # Adds rrt branch lines to the plot
    line_plotter.plot_branches(axis[1], branch_set, 'orange')
    # Adds rrt path lines to the plot
    line_plotter.plot_lines(axis[1], path, 'red')
    # Plotting the image with a grayscale colormap
    axis[1].imshow(world_map_rgb, cmap='gray')
    # Specifying the plot title
    axis[1].set_title('RRT run time: ' + format(computation_time, '.2f') +
                      ' seconds, path length: ' + format(path_length, '.2f'))
    # Specifying the label for the x-axis
    axis[1].set_xlabel('Map Columns')
    # Specifying the label for the y-axis
    axis[1].set_ylabel('Map Rows')
    # We print the results on terminal
    print('RRT - The path length is: ' + format(path_length, '.2f') + ', the computation time is:' +
          format(computation_time, '.2f') + ', the number of nodes in RRT tree is: ' + repr(node_count) +
          ' and the number of edges in the RRT tree is: ' + repr(len(branch_set)))

    # ---------- Integrated PRM Algorithm ----------
    # Uses the integrated PRM technique to find a path to the destination
    path, path_length, computation_time, roadmap_edge_list = integrated_prm.find_path(world_map, dynamic_obstacle_list,
                                                                                      start_coordinate, end_coordinate)
    # We mark the nodes of the integrated prm roadmap using certain icons
    line_plotter.point_marker(axis[2], integrated_prm.roadmap_node_list, mark_color='b')
    # We mark the edges of the roadmap onto the world map. Edges over dynamic obstacles will be shown in dotted fashion
    line_plotter.plot_edges_prm(axis[2], roadmap_edge_list)
    # Adds PRM path lines to the plot
    line_plotter.plot_lines(axis[2], path, 'red')
    # Plotting the image with a grayscale colormap
    axis[2].imshow(world_map_rgb, cmap='gray')
    # Specifying the plot title
    axis[2].set_title('Integrated PRM run time: ' + format(computation_time, '.2f') +
                      ' seconds, path length: ' + format(path_length, '.2f'))
    # Specifying the label for the x-axis
    axis[2].set_xlabel('Map Columns')
    # Specifying the label for the y-axis
    axis[2].set_ylabel('Map Rows')
    # We print the results on terminal
    print('Integrated PRM - The path length is: ' + format(path_length, '.2f') + ', the computation time is:' +
          format(computation_time, '.2f') + ', the number of nodes is: ' + repr(len(integrated_prm.roadmap_node_list)) +
          ' and the number of roadmap edges is :' + repr(len(roadmap_edge_list)))

    # Display the plot
    plot.show()

