#!/usr/bin/env python3


import sys
import math
import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan



LINEAR_VELOCITY = 0.2
ANGULAR_VELOCITY = 0.4
TOLERANCE = 0.3
ROBOT_RADIUS = 0.22
OBSTACLE_THRESHOLD = 0.78
EXIT_STATUS_ERROR = 1
EXIT_STATUS_OK = 0



class goto:
    def __init__(self):
        rospy.init_node('traveller',anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.laser_subscriber = rospy.Subscriber('/scan',self.laser_callback)
        self.sonar_subscriber = rospy.Subscriber('/r1/pseudosonar/scan',LaserScan,self.sonar_callback)

        #hold position and quaternion of robot
        self.pos = Pose()
        self.theta = 0
        self.obstacle_found = False
        self.obstacle_circumventing = False
        self.start = (0,0)
        self.goal = None
        self.mline = None
        self.curr_line = None
        #self.sonar_data = []
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)



    def odom_callback(self,odom):
        """
        Callback function for odometry subscriber, which will continously update the current position of the robot
        """

        self.pos = odom.pose.pose
        self.theta = 2 * math.atan2(self.pos.orientation.z, self.pos.orientation.w)
        self.curr_line = self.slope((self.pos.position.x, self.pos.position.y),self.goal)


    def laser_callback(self,laser_data):
        """
        Callback function for the laser subscriber, which will continuously update the current status of whether an obstacle is detected or not
        """
        # check if laser_data.ranges[321:350] is not infinity
        region = laser_data.ranges[321:350]
        m = float('inf')
        for d in region:
            if d < OBSTACLE_THRESHOLD and not self.obstacle_circumventing:
                self.obstacle_found = True
                self.stop()
                print("Obstacle detected.")
                break


    def sonar_callback(self,sonar_data):
        """
		Callback function for the sonar subscriber, which will continuously update the 
		current status of sonar obstacle detection data
		@arg 	sonar_data 	LaserScan object with a list "range" of range 0-682
		"""
        self.sonar_data = sonar_data.ranges

    def slope(self,p1,p2):
        """
        Method to calculate the slope of a line segment formed by two given points
        p1 first point of line segment
        p2 second point of line segment
        """
        delta_y = p2[1] - p1[1]
        delta_x = p2[0] - p1[0]
        return delta_y / delta_x if delta_x!=0 else float('inf')

    def euclidean_distance(self):
        """
        Method to compute distance from current position to the goal
        """
        return math.sqrt(math.pow(self.goal[0] - self.pos.position.x),2) + math.pow((self.goal[1] - self.pos.position.y),2)

    def angular_distance(self):
        """
        Method to compute absolute difference between the desired angle the current
        """
        return math.atan2(self.goal[1] - self.pos.position.y, self.goal[0] - self.pos.position.x) - self.theta

    def stop(self):
        """
        Method to bring robot to a halt by publishing linear and angular velocities of zero
        """ 
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_publisher.publish(self.vel_msg)


    """
    # not understood
    def bug(self):
        
        #Method to climb to obstacle by going around it
        

        self.obstacle_circumventing = True
        #min_sonar = min(self.sonar_data)
        LEFT90 = 600
        # rotate in place until sonar on left or robot +90 degrees is along tangent of obstacle surface
        while abs(self.sonar_data[LEFT90] - min_sonar)

    """


