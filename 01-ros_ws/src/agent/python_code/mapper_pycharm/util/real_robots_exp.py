import numpy as np
from copy import copy
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped, Pose,Quaternion

def adjust_pose(pose):
    """

    :type pose: Pose
    """
    new = copy(pose)
    angles = euler_from_quaternion([new.orientation.x, new.orientation.y, new.orientation.z, new.orientation.w])
    angles = (0, 0, angles[2])
    new.orientation = Quaternion(*quaternion_from_euler(*angles))
    # print [a/np.pi*180 for a in angles]
    new.position.z = 0.

    return new

def map_trans_orient_to_PoseStamped(trans,orient):
    new_PoseStamped = PoseStamped()
    new_PoseStamped.pose.position.x = trans[0]
    new_PoseStamped.pose.position.y = trans[1]
    new_PoseStamped.pose.position.z = trans[2]
    new_PoseStamped.pose.orientation.x = orient[0]
    new_PoseStamped.pose.orientation.y = orient[1]
    new_PoseStamped.pose.orientation.z = orient[2]
    new_PoseStamped.pose.orientation.w = orient[3]
    return new_PoseStamped