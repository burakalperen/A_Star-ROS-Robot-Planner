#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback
from geometry_msgs.msg import PoseStamped
import tf2_ros
from nav_msgs.msg import Path,Odometry
from nav_msgs.srv import GetPlan
import utils

class MoveBase():

    def __init__(self):
        rospy.init_node('move_base')

        # this is the service for listen a new goal from client.
        self.action_s = actionlib.SimpleActionServer('move_base',MoveBaseAction,
                        execute_cb=self.execute_cb,auto_start=False)


        # Get start pose from odom topic to send start_pose global planner
        rospy.init_node('get_startPose')
        rospy.Subscriber("odom",Odometry,self.odom_cb)
        self.odom = None
        self.start_pose = None

        # Stores known frames and optionally offers a ROS service, "tf2_frames", which responds to client requests
        # with a response containing a :class:`tf2_msgs.FrameGraph` representing the relationship of
        #known frames. 
        self.tf_buffer = tf2_ros.Buffer()
        # class:`TransformListener` is a convenient way to listen for coordinate frame transformation info.
        # This class takes an object that instantiates the :class:`BufferInterface` interface, to which
        # it propagates changes to the tf frame graph.
        tf2_ros.TransformListener(self.tf_buffer)


        # bu callback fonksiyonunun amacı action-client yapısında olmayan goal isteklerini almak
        #rospy.Subscriber('move_base_simple/goal',PoseStamped, self.goal_callback)


        # publish path
        self.path_pub = rospy.Publisher('path', Path, latch=True,queue_size=10)

        # create_global_plan adından bir servis oluşturuluyor
        rospy.loginfo("Waiting for create_global_plan service...")
        rospy.wait_for_service('create_global_plan')
        rospy.loginfo("OK")

        # global_plan oluştuktan sonra action başlıyor ve execute_cb fonksiyonuna üşüyor
        self.action_s.start()
        rospy.loginfo("Move Base Service started....")
        rospy.spin()


    def execute_cb(self,goal):
        # this is the callback that occur when a new goal arrives through the simple actin client.

        # store the goal pose in the global frame so we can monitor progress
        # goal_pose_world = pose_transform(self.tf_buffer,goal.target_pose,'map')
        
        rospy.loginfo("Target is received!")
        #rospy.loginfo("Goal: ",goal)
        #print("Goal: ",goal)    


        # goal will ha a .target_pose field
        # go to request_plan function for path
        path = self.request_plan(goal.target_pose)
        
        if path is None:
            msg = "Global planner couldn't find plan. Giving up."
            self.action_s.set_aborted(text=msg)
            return

        #self.action_s.set_succeeded()
        #print("Another: ",path.plan)
        rospy.loginfo(path.plan)
        self.path_pub.publish(path.plan)


    def request_plan(self,goal_pose):
        # block until a service is available.
        rospy.loginfo("Waiting for create_global_plan service...")
        rospy.wait_for_service('create_global_plan')

        try:
            robot_pose = self.start_pose
            if robot_pose == None:
                rospy.loginfo("Robot_pose is None.")
                return
            robot_pose = self.convert_pose(robot_pose) #to provide necessary conversions

            get_plan = rospy.ServiceProxy('create_global_plan',GetPlan)
            response_plan = get_plan(robot_pose,goal_pose,.15) # .15 is xy_goal_tolerance

            rospy.loginfo("Plan received.")
            #print("response_plan: ",response_plan.plan)
            #rospy.loginfo(response_plan)
            # if len(response_plan.poses == 0):
            #     response_plan = None
            
        except:
            response_plan = None

        return response_plan 

    # def get_current_pose(self):
    #     """ Return the current robot pose in the world frame. """
    #     pose_base = create_pose_stamped(0, 0, frame_id='base_link')
    #     return pose_base
    #     #return pose_transform(self.tf_buffer, pose_base, 'map')


    def odom_cb(self,odom):
        self.odom = odom
        start_pose = PoseStamped()
        start_pose.header.stamp = odom.header.stamp 
        start_pose.header.frame_id = odom.header.frame_id
        start_pose.pose.position = odom.pose.pose.position
        start_pose.pose.orientation = odom.pose.pose.orientation
                
        self.start_pose = start_pose

    def convert_pose(self,pose):
        tf_robotpose = utils.pose_to_tf2(pose) # convert a geometry_msgs/PoseStamped object to tf2_geometry_msgs/PoseStamped object
        map_pose = utils.pose_transform(self.tf_buffer,tf_robotpose,"map") # to transform provided pose into the target coordinate frame 
        pose = utils.pose_to_tf1(map_pose) # convert a tf2_geometry_msgs/PoseStamped object to a geometry_msgs/PoseStamped object
        return pose


if __name__ == "__main__":
    MoveBase()