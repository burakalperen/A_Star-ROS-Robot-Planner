#!/usr/bin/env python3

from inspect import cleandoc
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



def client():


    rospy.init_node("send_goal")
    client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    print("Waiting for sending goal to service...")
    client.wait_for_server()
    target = MoveBaseGoal()
    target.target_pose.header.frame_id = 'map'
    target.target_pose.pose.position.x = 0.6
    target.target_pose.pose.position.y = -0.5
    #target.target_pose.pose.position.y = 2.0
    client.send_goal(target)
    print("Goal send.")
    
    #rospy.spin()
    client.wait_for_result()
    print('Hedefe varildi.')



client()


