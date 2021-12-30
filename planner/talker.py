#!/usr/bin/env python3 


import rospy
from std_msgs.msg import String



"""listen odometry, its required to get one time."""

class talker():
    def __init__(self):
        
        publisher = rospy.Publisher('chatter',String,queue_size=10)
        rospy.init_node('talker',anonymous=True)

        while not rospy.is_shutdown():
            rospy.loginfo("Talker is active.")
            hello_str = "hello"
            publisher.publish(hello_str)
            rospy.spin()






if __name__ == "__main__":
    talker()










