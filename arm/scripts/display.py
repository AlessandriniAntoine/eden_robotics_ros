#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped,PointStamped,Point

"""
This node display the reference and the measure. It transform a tf topic from rviz into a PoseStamped
"""

def cb(event):
    global point
    stamp = rospy.Time.now()
    try:
        listener.waitForTransform(src_frame, dst_frame, stamp,
                                timeout=rospy.Duration(1))
    except Exception as e:
        rospy.logerr(e)
        return

    dst_pose = listener.lookupTransform(src_frame, dst_frame, stamp)

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = src_frame
    pose_msg.header.stamp = stamp
    pose_msg.pose.position.x = dst_pose[0][0]
    pose_msg.pose.position.y = dst_pose[0][1]
    pose_msg.pose.position.z = dst_pose[0][2]
    pose_msg.pose.orientation.x = dst_pose[1][0]
    pose_msg.pose.orientation.y = dst_pose[1][1]
    pose_msg.pose.orientation.z = dst_pose[1][2]
    pose_msg.pose.orientation.w = dst_pose[1][3]

    pub_state.publish(pose_msg)

    point_msg = PointStamped()
    point_msg.header.stamp = stamp
    point_msg.point.x = point[0]
    point_msg.point.y = point[1]
    point_msg.point.z = point[2]

    pub_state.publish(pose_msg)
    pub_ref.publish(point_msg)

def on_receive_callback(data):
    global point
    point = np.array([data.x,data.y,data.z])

if __name__ == '__main__':
    rospy.init_node('display_position_node')
    pub_state = rospy.Publisher('/Robot/state/measure', PoseStamped, queue_size=10)
    pub_ref = rospy.Publisher('/Robot/state/reference',PointStamped,queue_size=10)
    sub_ref = rospy.Subscriber('/Robot/ref/position',Point,on_receive_callback,queue_size=10)

    point = np.array([0,0,0])
    src_frame = 'origin'
    dst_frame = 'target'
    try:
        rate = rospy.get_param('/rate')
    except :
        rate = 100

    listener = tf.TransformListener()
    timer = rospy.Timer(rospy.Duration(1.0 / rate), cb)
    
    rospy.spin()
