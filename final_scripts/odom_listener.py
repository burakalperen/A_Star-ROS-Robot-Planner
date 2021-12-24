#!/usr/bin/env python3 


import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf2_ros
import utils
"""listen odometry, its required to get one time."""


class odom_listener():
    def __init__(self):
        rospy.init_node("listener",anonymous=True)
        rospy.Subscriber('odom',Odometry,self.cb)
        #rospy.spin()

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0) # accumulate some transforms        

        self.start_pose = None

        rospy.sleep(4.0)
        #rospy.spin()

        while not rospy.is_shutdown():

            robot_pose = self.start_pose
            if robot_pose == None:
                rospy.loginfo("Robot_pose is None.")
                pass
            robot_pose = self.convert_pose(robot_pose) #to provide necessary conversions
            
            #print("Robot pose: ",robot_pose)


    def cb(self,odom):
        self.odom = odom
        
        start_pose = PoseStamped()
        start_pose.header.stamp = odom.header.stamp 
        start_pose.header.frame_id = odom.header.frame_id
        start_pose.pose.position = odom.pose.pose.position
        start_pose.pose.orientation = odom.pose.pose.orientation
                
        self.start_pose = start_pose


    def convert_pose(self,pose):
        #print("TF_robotpose: ",pose)
        tf_robotpose = utils.pose_to_tf2(pose) # convert a geometry_msgs/PoseStamped object to tf2_geometry_msgs/PoseStamped object
        map_pose = utils.pose_transform(self.tf_buffer,tf_robotpose,"map") # to transform provided pose into the target coordinate frame 
        pose = utils.pose_to_tf1(map_pose) # convert a tf2_geometry_msgs/PoseStamped object to a geometry_msgs/PoseStamped object
        print("Tf_pose: ",pose)
        
        return pose


if __name__ == "__main__":
    odom_listener()








