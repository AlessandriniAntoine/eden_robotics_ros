#!/usr/bin/env python3

import time
import rospy


from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from kinematics.parameters import *

class Change_Frame_Node:
    """
    Node for the forward kinematics of the arm
    """

    def __init__(self,rosName="change_frame_node"):

        # Init ROS node
        rospy.init_node(rosName, anonymous=True)
        try:
            rate = rospy.get_param('/rate')
        except :
            rate = 100
        self.rosRate = rospy.Rate(rate)

        # init variables
        self.config_init = m_c

        self.screw_list = screw_list
        self.thetalist = np.array([0,0,0,0])

        self.point_s = qe # point in base frame
        self.point_b = translation_ce # point in end effector frame
        self.point_b_p = translation_ce # point in end effector frame at time n-1

        # init publisher and subscriber
        self.pub = rospy.Publisher('/Robot/ref/position',Point,queue_size=10)
        time.sleep(1)
        self.publish()
        self.rosRate.sleep()
        time.sleep(5)
        self.initGraph()

    def initGraph(self):
        self.sub_joint = rospy.Subscriber('/Robot/state/joint',JointState,self.on_receive_callback_joint,queue_size=10)
        self.sub_position = rospy.Subscriber('/Camera/ref/position',Point,self.on_receive_callback_point,queue_size=10)

    def on_receive_callback_joint(self,data):
        self.thetalist = np.array(data.position)

    def on_receive_callback_point(self,data):
        self.point_b = np.array([data.x,data.y,data.z,1])

    def publish(self):
        msg = Point()
        msg.x = self.point_s[0]
        msg.y = self.point_s[1]
        msg.z = self.point_s[2]
        self.pub.publish(msg)

    def updateConfig(self):
        while not rospy.is_shutdown():
            config = mr.FKinSpace(self.config_init,self.screw_list,self.thetalist)
            self.point_s = np.dot(config,self.point_b)
            diff = self.point_b_p-self.point_b
            if any(diff):
                self.publish()
                self.rosRate.sleep()
                self.point_b_p = self.point_b.copy()


def main():
    frame = Change_Frame_Node()
    frame.updateConfig()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass