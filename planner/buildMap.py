#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import math


class Map:
    """
	The Map class - this builds a map from a given map image
	Given map image is a binary image - it is already an occupancy grid map
	Coordinates must be converted from pixels to world when used
	For each pixel on the map, store value of the pixel - true if pixel obstacle-free, 
	false otherwise
	"""

    def __init__(self):
        """
        Construct an occupancy grid map from the image
        """

        self.map_image = Image.open("Map.png")
        self.width, self.height = self.map_image.size
        self.pixels = self.map_image.load()
        self.grid_map = []
        for x in range(self.width):
            # x: 0 - 2000
            
            row = []
            for y in range(self.height):
                if self.pixels[x,y] == 0:
                    row.append(False)
                else:
                    row.append(True)
            self.grid_map.append(row)


if __name__ == "__main__":
    Map()