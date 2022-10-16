#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from kinematics.parameters import *

class Forward_Node:
    """
    Node for the forward kinematics of the arm
    """

    def __init__(self,rosName="forward_node",rate=100):


        # Init ROS node
        rospy.init_node(rosName, anonymous=True)
        try:
            rate = rospy.get_param('/rate')
        except :
            rate = rate
        self.rosRate = rospy.Rate(rate)

        # init variables
        self.config_init = m_e
        self.config = m_e
        self.screw_list = screw_list
        self.thetalist = np.array([0,0,0,0])
        self.point = np.array([0,0,0,0])

        # init publisher and subscriber
        self.initGraph()

    def initGraph(self):
        self.pub = rospy.Publisher('EndEffector/state/position',Point,queue_size=10)
        self.sub = rospy.Subscriber('/Robot/ref/joint',JointState,self.on_receive_callback,queue_size=10)

    def on_receive_callback(self,data):
        self.thetalist = np.array(data.position)

    def publish(self):
        msg = Point()
        msg.x = self.point[0]
        msg.y = self.point[1]
        msg.z = self.point[2]
        self.pub.publish(msg)

    def updateConfig(self):
        while not rospy.is_shutdown():
            self.config = mr.FKinSpace(self.config_init,self.screw_list,self.thetalist)
            self.point = self.config.dot(np.array([0,0,0,1]))[:-1]
            self.publish()
            self.rosRate.sleep()

def main():
    forward = Forward_Node()
    forward.updateConfig()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass