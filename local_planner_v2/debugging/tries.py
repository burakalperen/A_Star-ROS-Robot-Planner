#!/usr/bin/env python3 

import rospy
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped,Twist
from time import time


def test_path():

    path = Path()

    first_point = PoseStamped()
    second_point = PoseStamped()
    third_point = PoseStamped()


    first_point.header.frame_id = "map"
    first_point.pose.position.x = -1.69
    first_point.pose.position.y = -0.499987
    first_point.pose.position.z = -0.001007


    second_point.header.frame_id = "map"
    second_point.pose.position.x = -1.39
    second_point.pose.position.y = -0.499987
    second_point.pose.position.z = -0.001007

    third_point.header.frame_id = "map"
    third_point.pose.position.x = -1.09
    third_point.pose.position.y = -0.499987
    third_point.pose.position.z = -0.001007


    path.poses.append(first_point)
    path.poses.append(second_point)
    path.poses.append(third_point)


    for a in path.poses:
        print(a.pose.position.x)


class Twist_publisher:
    def __init__(self):
        rospy.init_node("twist_publisher")
        self.vel_publisher = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

        self.velocity = Twist()
        self.velocity.linear.x = 0.5
        
        begin_time = time()
        while time() - begin_time < 2:
            self.vel_publisher.publish(self.velocity)


        

    def set(self):
        self.velocity = Twist()
        self.velocity.linear.x = 0.1
        self.vel_publisher.publish(self.velocity)

class Path_publisher:
    def __init__(self):
        rospy.init_node("path_publisher")
        path_publisher = rospy.Publisher("/path",Path,queue_size=10)

        while not rospy.is_shutdown():
            path = self.contruct_path()
            path_publisher.publish(path)

    def contruct_path(self):
        path = Path()

        first_point = PoseStamped()
        second_point = PoseStamped()
        third_point = PoseStamped()


        first_point.header.frame_id = "map"
        first_point.pose.position.x = -1.69
        first_point.pose.position.y = -0.499987
        first_point.pose.position.z = -0.001007


        second_point.header.frame_id = "map"
        second_point.pose.position.x = -1.39
        second_point.pose.position.y = -0.499987
        second_point.pose.position.z = -0.001007

        third_point.header.frame_id = "map"
        third_point.pose.position.x = -1.09
        third_point.pose.position.y = -0.499987
        third_point.pose.position.z = -0.001007


        path.poses.append(first_point)
        path.poses.append(second_point)
        path.poses.append(third_point)

        return path


if __name__ == "__main__":

    # begin_time = time()
    # print(begin_time)

    # while time() - begin_time < 2:
    #     print("Current time: ",time())
    #     print("Start time: ",begin_time)
    #     print("Diff: ",time() - begin_time)

    Path_publisher()
