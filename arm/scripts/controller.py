#!/usr/bin/env python3

import numpy as np
import time
from kinematics.parameters import *

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

class Controller_Node:
    """
    Node for the forward kinematics of the arm
    """

    def __init__(self,rosName="controller_node",rate=100):

        # Init ROS2 node
        rospy.init_node(rosName, anonymous=True)
        self.rosRate = rospy.Rate(rate)
        self.initGraph()

        # init variables
        self.action = False # authorize the robot to move
        self.axes = [0 for i in range(8)]
        self.buttons = [0 for i in range(15)]        
        self.point_camera = np.array([0,0,0])


        # init parameters
        self.translation = translation_ce
        self.position_zero = qe

    def initGraph(self):
        self.sub_joy = rospy.Subscriber('/Controller/ref',Joy,self.on_receive_callback,queue_size=10)
        self.pub_camera = rospy.Publisher('Camera/ref/position',Point,queue_size=10)
        self.pub_zero = rospy.Publisher('Base/ref/position',Point,queue_size=10)

    def on_receive_callback(self,data):
        self.axes = data.axes
        self.buttons = data.buttons

    def publish_camera(self):
        msg = Point()
        msg.x = self.point_camera[0]
        msg.y = self.point_camera[1]
        msg.z = self.point_camera[2]
        self.pub_camera.publish(msg)

    def publish_zero(self):
        msg = Point()
        msg.x = self.position_zero[0]
        msg.y = self.position_zero[1]
        msg.z = self.position_zero[2]
        self.pub_zero.publish(msg)

    def update(self):
        while not rospy.is_shutdown():
            if self.buttons[0]:
                self.action = True
            elif self.buttons[1]:
                self.action = False
            elif self.buttons[4]:
                self.publish_zero()    
                time.sleep(10)
            if self.action:
                self.point_camera = self.translation + np.array([self.axes[1],self.axes[0],-self.axes[3]])
                self.publish_camera()
 
def main():
    controller = Controller_Node()
    controller.update()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass