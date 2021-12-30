#!/usr/bin/env python3 

import rospy
import tf2_geometry_msgs
from tf2_geometry_msgs import PoseStamped
import tf

def pose_transform(tf_buffer, pose, target_frame):
    """Use a properly initilized tf2 Buffer to transform the provided pose
    into the target coordinate frame.

    Args:
        tf_buffer -  a properly initilized tf2 Buffer
        pose -  geometry_msgs/PoseStamped object
        target_frame - string representing the target frame

    Returns: a geometry_msgs/Pose object or None if something went wrong

    """
    pose2 = pose_to_tf2(pose)

    try:
        pose2 = tf_buffer.transform(pose2, target_frame,
                                    rospy.Duration(.1))
        # convert to standard PoseStamped
        pose_new = pose_to_tf1(pose2)
        print("pose_new: ",pose_new)
        return pose_new

    except Exception as e:
        print(type(e))
        print(e)
        return None

def pose_to_tf1(pose):
    """Convert a tf2_geometry_msgs/PoseStamped object to a
    geometry_msgs/PoseStamped object.

    """
    new_pose = PoseStamped()
    new_pose.header.stamp = pose.header.stamp
    new_pose.header.frame_id = pose.header.frame_id
    new_pose.pose.position = pose.pose.position
    new_pose.pose.orientation = pose.pose.orientation
    return new_pose

def pose_to_tf2(pose):
    """Convert a geometry_msgs/PoseStamped object to a
    tf2_geometry_msgs/PoseStamped object.

    """
    new_pose = tf2_geometry_msgs.PoseStamped()
    new_pose.header.stamp = pose.header.stamp
    new_pose.header.frame_id = pose.header.frame_id
    new_pose.pose.orientation = pose.pose.orientation
    new_pose.pose.position = pose.pose.position
    return new_pose

