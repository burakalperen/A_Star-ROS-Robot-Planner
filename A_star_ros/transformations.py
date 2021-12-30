from costmap_config import COSTMAP_DICT
from math import sqrt,cos,sin,atan2

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


def worldtheta_to_pixeltheta(world_theta):
    x = 1 * cos(world_theta)
    y = 1 * sin(world_theta)
    # since the y axis is inverted in the map the angle will be different
    return atan2(-y, x)

def dist(point_a, point_b):
    return sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)

