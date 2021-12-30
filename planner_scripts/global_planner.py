#!/usr/bin/env python3 

import rospy
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanResponse
from buildMap import Map
from geometry_msgs.msg import PoseStamped


class global_planner():
    def __init__(self):
        rospy.init_node("global_planner_service")
        get_plan = rospy.Service("create_global_plan",GetPlan,self.planner_cb)
        print("Global planner service active.")
        rospy.spin()


    def planner_cb(self,req):
        # print("Global planner callback: ")
        # print("Start: ",req.start)
        # print("Goal: ",req.goal)
        # print("Tolerance: ",req.tolerance)

        #self.map = Map().grid_map

        path = self.construct_path(req.start,req.goal,req.tolerance)

        

        print("Path sended.")
        return path

    def construct_path(self,start,goal,tolerance):
        path = Path()

        first_point = PoseStamped()
        second_point = PoseStamped()
        third_point = PoseStamped()


        first_point.header.frame_id = "map"
        first_point.pose.position.x = 0.2
        first_point.pose.position.y = -0.5

        second_point.header.frame_id = "map"
        second_point.pose.position.x = 0.4
        second_point.pose.position.y = -0.5

        third_point.header.frame_id = "map"
        third_point.pose.position.x = 0.6
        third_point.pose.position.y = -0.5


        path.poses.append(first_point)
        path.poses.append(second_point)
        path.poses.append(third_point)
        
        return path








if __name__ == "__main__":
    global_planner()










"""
..................HATAYI INCELE OLDU GIBI.....................
def planner_callback(req):
    a = 2
    return a



def global_planner():
    rospy.init_node("global_planner_service")
    s = rospy.Service('/move_base/make_plan',GetPlan,planner_callback)
    print("Global planner servis aktif..")
    rospy.spin()




if __name__ == "__main__":
    global_planner()

"""