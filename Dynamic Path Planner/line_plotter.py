# This python file contains functions useful for image plotting purposes
# Created by Ashwin Vinoo
# Date: 3/9/2019

# importing the necessary modules
import matplotlib.lines as lines
import numpy as np
import copy


# This function helps to plot lines via the plot axis handle and the path provided list of - (y,x)
def plot_lines(axis, path, colour='red', line_style='-'):
    # Check if the path is empty
    if len(path) < 2:
        # Returns from the function
        return
    else:
        # Gets the first element of the path
        previous_coordinate = path[0]
        # Iterating from the second coordinate in the path till the last coordinate
        for coordinate in path[1:]:
            # Obtaining the line to be plotted
            line = lines.Line2D([previous_coordinate[1], coordinate[1]],
                                [previous_coordinate[0], coordinate[0]], color=colour, linestyle=line_style)
            # Adding the specified lines to the axis
            axis.add_line(line)
            # The previous coordinate is updated
            previous_coordinate = coordinate


# This function plots lines between sets of two points via the plot axis handle - list of [(y1,x1), (y2,x2)]
def plot_branches(axis, branches, colour='orange', line_style='-'):
    # Iterating through the coordinates in the branches list
    for coordinate in branches:
        # Obtaining the line to be plotted
        line = lines.Line2D([coordinate[0][1], coordinate[1][1]],
                            [coordinate[0][0], coordinate[1][0]], color=colour, linestyle=line_style)
        # Adding the specified lines to the axis
        axis.add_line(line)


# This function specializes in plotting the edges of the Integrated PRM
def plot_edges_prm(axis, roadmap_edge_list, colour='orange', line_style_free='-', line_style_blocked=':'):
    # Iterating through the edges in the roadmap edge list
    for edge in roadmap_edge_list:
        # We initialize the line style to the default
        line_pattern = line_style_free
        # Check if the edge is marked as blocked
        if edge.dynamic_obstacle_overlap:
            # We mark the line style as that of blocked
            line_pattern = line_style_blocked
        # Obtaining the line to be plotted. Line is plotted as dashes
        line = lines.Line2D([edge.vertex_1[1], edge.vertex_2[1]],
                            [edge.vertex_1[0], edge.vertex_2[0]], color=colour, linestyle=line_pattern, linewidth=0.75)
        # Adding the specified lines to the axis
        axis.add_line(line)


# Concatenate three (height, width) images into one (height, width, 3)
def concat_channels(r, g, b, multiplier=1):
    # In case the R, G and B components don't have matching dimensions
    if r.shape != g.shape or r.shape != b.shape:
        # Raise an exception
        raise Exception('The R, G and B components don\'t have matching dimensions')
    # We initialize the new RGB array
    rgb_array = np.zeros((r.shape[0], r.shape[1], 3), dtype=np.uint8)
    # The first element of third dimension is red channel
    rgb_array[..., 0] = np.minimum(np.multiply(r, multiplier), 255)
    # The second element of third dimension is red channel
    rgb_array[..., 1] = np.minimum(np.multiply(g, multiplier), 255)
    # The third element of third dimension is red channel
    rgb_array[..., 2] = np.minimum(np.multiply(b, multiplier), 255)
    # Returns the RGB array
    return rgb_array


# Marks in a specified color in the image at the coordinates specified within the list
def color_marker(image_rgb, coordinate_list, color):
    # Raises an exception if anything other than an RGB array is passed
    if len(image_rgb.shape) != 3 or image_rgb.shape[2] != 3:
        # Raise an exception
        raise Exception('Can\'t handle non RGB images')
    # We make a shallow copy of the rgb image in order to prevent permanent modification
    image = image_rgb.copy()
    # Iterating through the coordinate list
    for coordinate in coordinate_list:
        # We mark in the color specified at the coordinate
        image[coordinate[0], coordinate[1], :] = color
    # Returns the RGB image
    return image


# This function helps to mark specific points on an image with an icon
def point_marker(axis, coordinate_list, mark_color, mark_shape='o', mark_size=5):
    # The y-coordinates in order
    y_coordinates = [item[0] for item in coordinate_list]
    # The x-coordinates in order
    x_coordinates = [item[1] for item in coordinate_list]
    # We mark those coordinates onto the axis of the figure
    axis.scatter(x_coordinates, y_coordinates, s=mark_size, c=mark_color, marker=mark_shape)
