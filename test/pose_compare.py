#!/usr/bin/env python

import math
import atexit

import numpy as np
from scipy.spatial.transform import Rotation

import rospy
import tf2_ros

import std_msgs.msg as stdmsg
import geometry_msgs.msg as geomsg


class CompareNode(object):

    def __init__(self):
        rospy.init_node("pose_compare")
        rospy.loginfo(rospy.get_name() + ": Initialising node...")

        # Subscribers
        self.vel_msg = None
        self.vel_sub = rospy.Subscriber("vel_filtered", geomsg.PointStamped, self.vel_cb, queue_size=10)

        # Pubishers
        self.pos_diff_pub = rospy.Publisher("diff/pos", geomsg.Vector3, queue_size=10)
        self.pos_norm_pub = rospy.Publisher("diff/pos_norm", stdmsg.Float32, queue_size=10)
        self.vel_norm_pub = rospy.Publisher("vel_norm", stdmsg.Float32, queue_size=10)
        self.quat_diff_pub = rospy.Publisher("diff/quat", geomsg.Quaternion, queue_size=10)
        self.quat_norm_pub = rospy.Publisher("diff/quat_norm", stdmsg.Float32, queue_size=10)

        # Set up tf stuff
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)

        # tf frames
        self.odom_frame          = rospy.get_param("~odom_frame",          default="odom")
        self.base_frame_internal = rospy.get_param("~base_frame_internal", default="base_link")
        self.base_frame_external = rospy.get_param("~base_frame_external", default="base_link_test")

        # Wait for tf frames to be set up
        self.tf_buff.lookup_transform(self.base_frame_internal, self.odom_frame, rospy.Time(), timeout=rospy.Duration(5))
        self.tf_buff.lookup_transform(self.base_frame_external, self.odom_frame, rospy.Time(), timeout=rospy.Duration(5))

        # Wait for first velocity message
        rospy.wait_for_message("vel_filtered", geomsg.PointStamped, timeout=5)

        atexit.register(self.print_rmse)

        # Start online comparision
        self.pos_diff_list = []
        self.quat_diff_list = []
        self.timer = rospy.Timer(rospy.Duration(0.05), self.compare_pose)

    def vel_cb(self, vel):
        self.vel_msg = vel

    def compare_pose(self, timer_event):
        # Extract latest transform for both frames.
        pose_internal = self.tf_buff.lookup_transform(self.base_frame_internal, self.odom_frame, rospy.Time()).transform
        pose_external = self.tf_buff.lookup_transform(self.base_frame_external, self.odom_frame, rospy.Time()).transform

        # Difference in position.
        pos_internal = pose_internal.translation
        pos_external = pose_external.translation
        pos_diff = geomsg.Vector3(pos_external.x - pos_internal.x, pos_external.y - pos_internal.y, pos_external.z - pos_internal.z)
        pos_norm = math.sqrt(pos_diff.x**2 + pos_diff.y**2 + pos_diff.z**2)

        # Difference in orientation.
        quat_internal = pose_internal.rotation
        quat_external = pose_external.rotation
        quat_diff = (Rotation((quat_external.x, quat_external.y, quat_external.z, quat_external.w)) 
                     * Rotation((quat_internal.x, quat_internal.y, quat_internal.z, quat_internal.w)).inv()).as_quat()
        quat_norm = 2 * np.arctan2(np.linalg.norm(quat_diff[0:3]), abs(quat_diff[3]))

        # Velocity
        vel = self.vel_msg.point
        vel_norm = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

        # Publish for online plotting.
        self.pos_diff_pub.publish(pos_diff)
        self.pos_norm_pub.publish(pos_norm)
        self.quat_diff_pub.publish(geomsg.Quaternion(*quat_diff))
        self.quat_norm_pub.publish(quat_norm)
        self.vel_norm_pub.publish(vel_norm)

        # Save for offline analysis when node is shutdown.
        self.pos_diff_list.append(pos_diff)
        self.quat_diff_list.append(quat_diff)

    def print_rmse(self):
        pos_rmse = math.sqrt(np.mean([pos_diff.x**2 + pos_diff.y**2 + pos_diff.z**2 for pos_diff in self.pos_diff_list]))
        rospy.loginfo(rospy.get_name() + ": Position RMSE = {}".format(round(pos_rmse, 4)))

        quat_rmse = math.sqrt(np.mean([(2 * np.arctan2(np.linalg.norm(quat_diff[0:3]), abs(quat_diff[3])))**2 for quat_diff in self.quat_diff_list]))
        rospy.loginfo(rospy.get_name() + ": Orientation RMSE = {}".format(round(quat_rmse, 4)))

def quat_magnitude(q):
    return 2 * np.arctan2(np.linalg.norm(q[0:3]), abs(q[3]))

if __name__ == "__main__":
    node = CompareNode()
    rospy.spin()


