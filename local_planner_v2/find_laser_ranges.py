#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan




class Laser:
    def __init__(self):
        rospy.init_node("laser_listener")
        self.laser_subscriber = rospy.Subscriber('/scan',LaserScan,self.laserscan_callback)
        print("created.")


        rospy.spin()

    def laserscan_callback(self,laser_msg):
        #print("Laser callback girdi.")
        region = laser_msg.ranges[635:645]
        print(region)
        #region = list(region)
        m = float('inf')
       
        # for d in region:
        #     if d < OBSTACLE_THRESHOLD and not self.obstacle_circumventing:
        #         self.obstacle_found = True
        #         self.stop()
        #         rospy.loginfo("Obstacle detected.")       
        #         break



if __name__ == "__main__":

    Laser()

