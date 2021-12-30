#!/usr/bin/env python3


from os import path
import threading
import numpy as np
import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


import tf2_geometry_msgs



class LocalPlanner(object):
    """
    Handle path following
    """

    def __init__(self):
        rospy.init_node("local_planner")

        rospy.Subscriber('path',Path,self.path_callback)

        self.path = None


    def path_callback(self,path_msg):
        """
        Initialize pure pursuit when a new path is received.
        """

        if len(path_msg.poses == 0):
            rospy.loginfo("Empty path.")
            self.path = None

        else:
            self.path = path_msg
            self.path_xs = [pose.pose.position.x for pose in path_msg.poses]
            self.path_ys = [pose.pose.position.y for pose in path_msg.poses]

            rospy.loginfo("Path_xs: ",self.path_xs)
            rospy.loginfo("Path_ys: ",self.path_ys)


