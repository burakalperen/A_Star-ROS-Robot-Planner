#!/usr/bin/env python3 

import rospy
from sensor_msgs.msg import LaserScan
from time import time
from geometry_msgs.msg import Twist


OBSTACLE_THRESHOLD = 0.78

class ListenerLaser:
    def __init__(self):
        rospy.init_node("obstacle_circumventing")
        self.laser_subscriber = rospy.Subscriber('/scan',LaserScan,self.laserscan_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel",Twist,queue_size=10)


        self.vel_msg = Twist()
        self.obstacle_found = None
        self.obstacle_circumventing = False

        #rospy.spin()

    def laserscan_callback(self,laser_msg):
        #print("Laser callback girdi.")
        region = laser_msg.ranges[321:350]
        #print(region)
        #region = list(region)
        m = float('inf')
       
        for d in region:
            if d < OBSTACLE_THRESHOLD and not self.obstacle_circumventing:
                self.obstacle_found = True
                self.stop()
                rospy.loginfo("Obstacle detected.")       
                break

    def stop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_publisher.publish(self.vel_msg)  

    def bug(self):
        self.obstacle_circumventing = True
        a = 0
        start_time = time()
        while time() - start_time < 8:
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.1
            self.vel_publisher.publish(self.vel_msg)
            print("Obstacle circumventing..")
            
        self.stop()
        self.obstacle_found = False
        self.obstacle_circumventing = False



    def go(self):
        while not rospy.is_shutdown():
            if self.obstacle_found:
                print(self.obstacle_found)
                #self.stop()
                print("Bug fonksiyonuna yönlendiriliyor...")
                self.bug()
            else:
                print("Robot travelling...")

if __name__ == "__main__":
    planner = ListenerLaser()
    planner.go()