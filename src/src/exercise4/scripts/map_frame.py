#!/usr/bin/python3

import rospy
import tf2_ros

rospy.init_node('map_frame')

static_broadcaster = tf2_ros.StaticTransformBroadcaster()

from geometry_msgs.msg import TransformStamped

transform = TransformStamped()

transform.header.stamp = rospy.Time.now()
transform.header.frame_id = "map"
transform.child_frame_id = "map"

transform.transform.translation.x = 0.0
transform.transform.translation.y = 0.0
transform.transform.translation.z = 0.0

transform.transform.rotation.x = 0.0
transform.transform.rotation.y = 0.0
transform.transform.rotation.z = 0.0
transform.transform.rotation.w = 1.0

static_broadcaster.sendTransform(transform)