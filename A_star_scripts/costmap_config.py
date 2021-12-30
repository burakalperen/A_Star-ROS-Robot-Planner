"""

This file includes costmap paremeters.
Astar just uses resolution and origin parameters in dictionary.

@author: Burak Alperen Ayaz

"""

COSTMAP_DICT = {
                "RESOLUTION": 0.05, #the resolution of the map in meters/cell(pixel)

                "ORIGIN": (-100,-100,0), #the origin of the map in the global frame in meters, ignore yaw
                
                "IMAGE_WIDTH": 1000, #width of costmap image
                
                "IMAGE_HEIGHT": 1000, #height of costmap image 
                
                "MAP_WIDTH": 50, # width meters of map
                
                "MAP_HEIGHT": 50, # height meters of map 
                
                "LASER_MAX": 8.0
                }


"""
map1 parameters
"""
# RESOLUTION = 0.05
# IMAGE_WIDTH = 1000
# IMAGE_HEIGHT = 1000
# ORIGINS = (-100,-100)
# MAP_WIDTH = 50
# MAP_HEIGHT = 50

"""
map2 parameters
"""
# RESOLUTION = 0.0625
# IMAGE_WIDTH = 2000
# IMAGE_HEIGHT = 700
# ORIGINS = (0,0)
# MAP_WIDTH = 125.0
# MAP_HEIGHT = 43.75
