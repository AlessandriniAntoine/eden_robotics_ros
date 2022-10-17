#!/usr/bin/env python3

import numpy as np
import time
from kinematics.parameters import *

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

class Conversion_Node:
    """
    Node to convert joy command into position command
    """

    def __init__(self,rosName="conversion_node"):

        # Init ROS node
        rospy.init_node(rosName, anonymous=True)
        
        # get rate if set as param
        try:
            rate = rospy.get_param('/rate')
        except :
            rate = 100
        self.rosRate = rospy.Rate(rate)
        self.initGraph()

        # init variables
        self.action = False # authorize the robot to move
        self.axes = [0 for i in range(8)]
        self.buttons = [0 for i in range(15)] 
        self.index = [[0,1,4],[1,0,3]] # button index can be change if joystick is not xbox (index[0] : button index, index[1]: axes index) To find the index you can you the joy_node node and print the /joy topic
        self.point_camera = np.array([0,0,0])


        # init parameters
        self.translation = translation_ce[:3]
        self.point_zero = qe

    def initGraph(self):
        self.sub_joy = rospy.Subscriber('/Joystick/ref/joy',Joy,self.on_receive_callback,queue_size=10)
        self.pub_camera = rospy.Publisher('/Camera/ref/position',Point,queue_size=10)
        self.pub_zero = rospy.Publisher('/Robot/ref/position',Point,queue_size=10)

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
        msg.x = self.point_zero[0]
        msg.y = self.point_zero[1]
        msg.z = self.point_zero[2]
        self.pub_zero.publish(msg)

    def update(self):
        while not rospy.is_shutdown():
            if self.buttons[self.index[0][0]]:
                self.action = True
            elif self.buttons[self.index[0][1]]:
                self.action = False
            elif self.buttons[self.index[0][2]]:
                self.publish_zero()
                self.action = False    
                time.sleep(5)
            if self.action:
                self.point_camera = self.translation + np.array([self.axes[self.index[1][0]],self.axes[self.index[1][1]],-self.axes[self.index[1][2]]])*3e-2
                self.publish_camera()
 
def main():
    converter = Conversion_Node()
    converter.update()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
