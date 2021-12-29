#!/usr/bin/env python3 

import rospy
from time import time
import sys
import math
from geometry_msgs.msg import Twist,Pose,PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Path

LINEAR_VELOCITY = 0.1
ANGULAR_VELOCITY = 0.1
TOLERANCE = 0.3
ROBOT_RADIUS = 0.22
OBSTACLE_THRESHOLD = 0.78
EXIS_STATUS_ERROR = 1
EXIS_STATUS_OK = 0



class LocalPlanner():
    def __init__(self):
        rospy.init_node('local_planner',anonymous=True)
        self.odom_subscriber = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.laser_subscriber = rospy.Subscriber('/scan',LaserScan,self.laser_callback)
        self.vel_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.path_subscriber = rospy.Subscriber('/path',Path,self.path_callback)

        self.odom = None
        self.theta = 0
        self.obstacle_found = False
        self.obstacle_circumventing = False
        self.start = None
        self.goal = (0,0) 
        self.curr_line = None
        self.mline = None
        self.vel_msg = Twist()
        rospy.spin() # uncomment using for class, not class' function

    def path_callback(self,path_msg):
        """
        Callback function for path subscriber. 
        If path is received and not None, call travel function for robot moving.
        
        @argument path_msg: nav_msgs.msg/Path
        """
        if path_msg == None:
            rospy.loginfo("Path is not found.")
        
        else:
            rospy.loginfo("Path is valid. Calling path_travel function...")
            self.path_travel(path_msg)

    def odom_callback(self,odom_msg):
        """
        Callback function for the odometry subscriber, which will continuously update 
        the current position of the robot.

        @argument odom_msg: geometry_msgs.msg/Odometry
        """
        self.odom = odom_msg
        self.theta = 2 *  math.atan2(self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
        self.curr_line = self.slope((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y),self.goal)
     
    def laser_callback(self,laser_data):
        """
        Callback function for the laser subscriber, which will continuoulsy update the 
        current status of whether an obstacle is detected or not. 
        
        laser scan ranges: (-42 degree,42 degree), increment_size:0.06 degree
        total lasers: 1280
        @argument laser_data: sensor_msgs.msg/LaserScan
        """
        # check if laser_data.ranges[321:350] is not infinitiy
        region = laser_data.ranges[635:645]
        #self.region = region        
        m = float('inf')
        for d in region:
            if d < OBSTACLE_THRESHOLD and not self.obstacle_circumventing:
                self.obstacle_found = True
                self.stop()
                break
    
    def bug(self):
        """
        Method to climb the obstacle by going around it.
        """
        rospy.loginfo("Robot is climbing obstacle...")
        self.obstacle_circumventing = True

        start_time = time()
        while time() - start_time < 6:
            self.vel_msg.angular.z = 0.1
            self.vel_publisher.publish(self.vel_msg)

        print("Obstacle avoided.")
        self.stop()


        self.start = (self.odom.pose.pose.position.x,self.odom.pose.pose.position.y)
        self.obstacle_found = False
        self.obstacle_circumventing = False
        self.go()

    def euclidean_distance(self):
        """
        Method to compute distance from current position to the goal
        """ 
        return math.sqrt(math.pow((self.goal[0]- self.odom.pose.pose.position.x),2) + 
                        math.pow((self.goal[1] - self.odom.pose.pose.position.y),2))

    def angular_difference(self):
        """
        Method to compute the absolute difference between the desired angle and the 
        current angle of the robot.
        """
        return math.atan2(self.goal[1] - self.odom.pose.pose.position.y, self.goal[0] - self.odom.pose.pose.position.x) - self.theta

    def slope(self,p1,p2):
        """
        Method to calculate the slope of a line segment formed by two given points
        """
        delta_y = p2[1] - p1[1]
        delta_x = p2[0] - p1[0] 
        return delta_y/delta_x if delta_x!=0 else float('inf')

    def stop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_publisher.publish(self.vel_msg)

    def go(self):
        """ 
        Method to go to goal specified irrespective of the current positio and orientation
        of the robot.
        """
        print("Goal: ",self.goal)

        while self.euclidean_distance() > TOLERANCE and not self.obstacle_found:
            #set linear velocity
            self.vel_msg.linear.x = min(LINEAR_VELOCITY,
                                        LINEAR_VELOCITY * self.euclidean_distance())

            self.vel_msg.angular.z = min(ANGULAR_VELOCITY, ANGULAR_VELOCITY * self.angular_difference())


            print("Current coordinate: {},{}".format(self.odom.pose.pose.position.x,self.odom.pose.pose.position.y))
            #publish velocity
            self.vel_publisher.publish(self.vel_msg)
            
            
        if self.obstacle_found:
            self.stop()
            rospy.loginfo("Obstacle detected and stopped.")
            self.bug()
        else:
            self.stop()
            rospy.loginfo("Arrived at goal.")

    def travel(self,goals):
        """
        Method to accept goals with list[tuple] format.
        argument: goals: [(x1,y1),(x2,y2)...,(x3,y3)]
        """
        
        rospy.sleep(5.0)
        for goal in goals:
            self.goal = goal
            if self.odom != None:
                self.start = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
                self.mline = self.slope(self.start,goal)
                self.go()
            else:
                rospy.loginfo("Odometry is None, so robot can't travel.")


    def path_travel(self,path_msg):
        """
        Method to accept path.
        argument: path_msg: nav_msgs.msg/Path
        """

        for pose in path_msg.poses:
            goal = (pose.pose.position.x,pose.pose.position.y)
            self.goal = goal
            if self.odom != None:
                self.start = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
                self.mline = self.slope(self.start,goal)
                self.go()
            else:
                rospy.loginfo("Odometry is None, robot can't not travelling.")


        
    

if __name__ == "__main__":
    goals = [(-0.67,0.38)]
    # local_planner = LocalPlanner()
    #local_planner.travel(goals)
    LocalPlanner()