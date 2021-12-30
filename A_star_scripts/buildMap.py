#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import cv2
import numpy as np
from transformations import world_to_pixel


class Map:
    """
	The Map class - this builds a map from a given map image
	Given map image is a binary image - it is already an occupancy grid map
	Coordinates must be converted from pixels to world when used
	For each pixel on the map, store value of the pixel - true if pixel obstacle-free, 
	false otherwise
	"""

    def __init__(self,image_path):
        """
        Construct an occupancy grid map from the image
        """

        self.map_image = Image.open(image_path)
        self.width, self.height = self.map_image.size
        self.pixels = self.map_image.load()
        self.grid_map = []
        for x in range(self.width):        
            row = []
            for y in range(self.height):
                if self.pixels[x,y] == 0:
                    row.append(False)
                else:
                    row.append(True)
            self.grid_map.append(row)

    # visualize path on image after a_star 
    def visualize(image_path,path_points):
        mapImage = cv2.imread(image_path)

        for p1, p2 in zip(path_points, path_points[1:]):
                p1 =  world_to_pixel(p1)
                p2 =  world_to_pixel(p2)
                cv2.line(mapImage, tuple(p1), tuple(p2), (0, 0, 255), 1)

        cv2.imshow("Astar in costmap",mapImage)
        cv2.waitKey(0)

if __name__ == "__main__":
    Map()