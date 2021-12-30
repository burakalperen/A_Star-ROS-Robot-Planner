"""

Test scripts the transformations between map and world coordinates
Update will soon for functionality, that's hard-coded for now.

@author: Burak Alperen Ayaz

"""

from PIL import Image
import cv2
import numpy as np
from costmap_config import COSTMAP_DICT




def world_to_pixel(world_points):
    world_x, world_y = world_points
    pixel_points = []

    pixel_points.append(int((world_x - COSTMAP_DICT["ORIGIN"][0]) / COSTMAP_DICT["RESOLUTION"]))
    pixel_points.append(int((world_y - COSTMAP_DICT["ORIGIN"][1]) / COSTMAP_DICT["RESOLUTION"]))
    return pixel_points

def pixel_to_world(pixel_points):
    pixel_x,pixel_y = pixel_points
    world_points = []

    world_points.append(pixel_x * COSTMAP_DICT["RESOLUTION"] + COSTMAP_DICT["ORIGIN"][0])
    world_points.append(pixel_y * COSTMAP_DICT["RESOLUTION"] + COSTMAP_DICT["ORIGIN"][1])
    return world_points



if __name__ == "__main__":
    
    img = cv2.imread("./map.png")
    

    pixel_points = (513,516)
    x1,y1 = pixel_to_world(pixel_points=pixel_points)
    print("World_points: ({},{})".format(x1,y1))

    pixel_points = (491,442)
    x2,y2 = pixel_to_world(pixel_points=pixel_points)
    print("World_points: ({},{})".format(x2,y2))


    world_points = (7.75,15.3125)
    x,y = world_to_pixel(world_points = world_points)
    print("Pixel points: ({},{})".format(x,y))


    cv2.imshow("img",img)
    cv2.waitKey(0)


