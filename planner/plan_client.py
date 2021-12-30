#!/usr/bin/env python3 
from os import stat
import rospy
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan

from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist,PoseStamped


def receive_plan(start,goal,tolerance):
    rospy.wait_for_service('/move_base/make_plan')


    get_plan = rospy.ServiceProxy("/move_base/make_plan",GetPlan)
    answer = get_plan(start,goal,tolerance)
    #print("Answer: ",answer)
    print("Servisle bağlanti kuruldu")
    return answer
    #except:
     #   print("Service call failed.")




if __name__ == "__main__":


    start = PoseStamped()

    start.header.frame_id = "map"
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0
    start.pose.position.z = 0.0
    

    goal = PoseStamped()

    goal.header.frame_id = "map"
    goal.pose.position.x = 3.0
    goal.pose.position.y = 0.0
    goal.pose.position.z = 0.0
    

    # start = (2,2)
    # goal = (3,3)
    tolerance = 0.2

    answer = receive_plan(start = start,goal = goal,tolerance = tolerance)
    print(answer)













"""
.......................HATAYI INCELE OLDU GIBI...........................
def receive_plan(a):
    rospy.wait_for_service('/move_base/make_plan')


    get_plan = rospy.ServiceProxy("/move_base/make_plan",GetPlan)
    answer = get_plan(a)
    #print("Answer: ",answer)
    print("Servisle bağlanti kuruldu")
    return answer
    #except:
     #   print("Service call failed.")




if __name__ == "__main__":

    print("Answer: ",receive_plan(2))

"""