#!/usr/bin/env python

from yaml import Mark
import rospy
import tf.transformations
import numpy as np

import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

marker_pose = PoseStamped()
pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def publisher_callback(msg, link_name):
    marker_pose.header.frame_id = link_name
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)

    marker.header.stamp = rospy.Time(0)
    marker_pub.publish(marker)


def wait_for_initial_pose():
    msg = rospy.wait_for_message("franka_state_controller/franka_states",
                                 FrankaState)  # type: FrankaState

    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / \
        np.linalg.norm(initial_quaternion)
    marker_pose.pose.orientation.x = initial_quaternion[0]
    marker_pose.pose.orientation.y = initial_quaternion[1]
    marker_pose.pose.orientation.z = initial_quaternion[2]
    marker_pose.pose.orientation.w = initial_quaternion[3]
    marker_pose.pose.position.x = msg.O_T_EE[12]
    marker_pose.pose.position.y = msg.O_T_EE[13]
    marker_pose.pose.position.z = msg.O_T_EE[14]

def move_callback():
    marker_pose.pose.position.x = math.sin(rospy.Time().now().to_sec()/1) * 0.25 + 0.1 + initial_x
    marker_pose.pose.position.y = math.cos(rospy.Time().now().to_sec()/1) * 0.25
    marker_pose.pose.position.z = math.sin(rospy.Time().now().to_sec()/1) * 0.125 + initial_z
    marker.pose.position.x = math.sin(rospy.Time().now().to_sec()/1) * 0.25 + 0.1 + initial_x
    marker.pose.position.y = math.cos(rospy.Time().now().to_sec()/1) * 0.25# + initial_y
    marker_pose.pose.position.z = math.sin(rospy.Time().now().to_sec()/1) * 0.125 + initial_z

if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")

    wait_for_initial_pose()

    initial_x = marker_pose.pose.position.x
    initial_y = marker_pose.pose.position.y
    initial_z = marker_pose.pose.position.z

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)

    pose_pub = rospy.Publisher(
        "equilibrium_pose", PoseStamped, queue_size=10)

    marker = Marker()
    marker.header.frame_id = link_name
    marker.ns = "m2"
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = .1
    marker.scale.y = .1
    marker.scale.z = .1
    marker.color.r = 0.1
    marker.color.g = 0.1
    marker.color.b = 0.7 
    marker.color.a = 0.7
    marker.action = Marker.ADD

    marker.pose = marker_pose.pose
    
    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisher_callback(msg, link_name))

    rospy.Timer(rospy.Duration(1./10), lambda _: move_callback())

    rospy.spin()
