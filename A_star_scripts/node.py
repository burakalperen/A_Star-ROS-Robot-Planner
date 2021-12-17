#!/usr/bin/env python3



import math
from typing import NamedTuple
from transformations import world_to_pixel, pixel_to_world, worldtheta_to_pixeltheta


SIMILARITY_THRESHOLD = 0.1
SAFETY_OFFSET = 5 # number of pixels away from the wall the robot should main


class Node:
    def __init__(self,x,y,theta=0.0,parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent

        self.f = 0
        self.h = 0
        self.g = 0

    
    def euclidean_distance(self,goal):
        """
        Method to compute distance from current position to the goal
        """
        return math.sqrt(math.pow((goal.x - self.x),2) + math.pow((goal.y - self.y),2))

    def apply_move(self,move):
        """
        Apply the given move to current position
        """
        theta_new = self.theta + move[1]
        x_new = self.x + math.cos(theta_new) * move[0]
        y_new = self.y + math.sin(theta_new) * move[0]
        return Node(x_new,y_new,theta_new)


    def is_move_valid(self,grid_map,move):
        """
        Return true if required move is legal
        """

        goal = self.apply_move(move)
        # convert goal coordinates to pixel coordinates before checking data
        goal_pixel = world_to_pixel((goal.x,goal.y), (700,2000))
        # check if too close to the walls
        if goal_pixel[0] >= SAFETY_OFFSET and not grid_map[int(goal_pixel[0]-SAFETY_OFFSET)][int(goal_pixel[1])]:
            return False
        if goal_pixel[1] >= SAFETY_OFFSET and not grid_map[int(goal_pixel[0])][int(goal_pixel[1]-SAFETY_OFFSET)]:
            return False   
        if goal_pixel[0] >= SAFETY_OFFSET and goal_pixel[1] >= SAFETY_OFFSET and not grid_map[int(goal_pixel[0]-SAFETY_OFFSET)][int(goal_pixel[1]-SAFETY_OFFSET)]:
            return False
        if grid_map[int(goal_pixel[0])][int(goal_pixel[1])]:
            return True
        
        return False


    def is_valid(self,grid_map):
        """
        Return true if the location on the map is valid, in obstacle free zone
        """
        print("\n********WORLD_TO_PIXEL*********")
        print("World x: ", self.x, "Pixel y: ",self.y)
        goal_pixel = world_to_pixel((self.x,self.y),(700,2000))
        print("Pixel x: ", goal_pixel[0], "Pixel y: ",goal_pixel[1])
        print("********WORLD_TO_PIXEL*********\n")

        # grid mapde her sütun için bir liste bulunuyor
        # bu listelerde her sütundaki satır elemanı için
        print("**********IS_VALID**************")
        print("Hedef pixel bos mu:  ",grid_map[int(goal_pixel[0])][int(goal_pixel[1])])
        print("**********IS_VALID**************\n")
        
        if grid_map[int(goal_pixel[0])][int(goal_pixel[1])]:
            return True
        else:
            return False

    def is_similar(self,other):
        """
        Return true if other node is in similar position as current node
        """

        return self.euclidean_distance(other) <= SIMILARITY_THRESHOLD