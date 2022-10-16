#!/usr/bin/env python3

import rospy
import math
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from submodules.motors import * 

class ROS_Dynamixel_node:
    """
    Create a ros node for the motors
    Parameters :
        - rosName : name for the node
        - rate : rate message in Hz
    """

    def __init__(self,rosName="motors_dynamixel_node",rate = 1/0.01) :

        # Init ROS
        rospy.init_node(rosName, anonymous=True)
        self.rosRate = rospy.Rate(rate)


        # set parameters
        self.ids = [1,2,3,4] # dynamixel motors id
        self.gearRatio = np.array([80/20,110/25,105/21,50/25])
        self.orientation = np.array([1,1,-1,1]) #to inverse rotation of motor 3

        # # deduce parameters
        self.numMotors = len(self.ids)

        # init variables
        self.goalPositions = np.array([0 for _ in self.ids])
        self.rotations = np.array([0 for _ in self.ids])
        self.current = np.array([0 for _ in self.ids])
        self.present_position = np.array([0 for _ in self.ids])

        # init motors
        self.motors =  MotorsSync(self.ids)
        self.initPosition = self.motors.read_position()
        
        # init topics
        self.initgraph()

    ############################## 
    # ROS Function
    ##############################

    def initgraph(self):
        self.pub_current = rospy.Publisher('/Motors/state/current',JointState,queue_size=10)
        self.pub_pp = rospy.Publisher('/Motors/state/present_position',JointState,queue_size=10)
        self.sub_joint = rospy.Subscriber('/Robot/state/joint',JointState,self.on_receive_callback_joint,queue_size=10)

    def publish_current(self):
        header = Header()
        header.stamp = rospy.Time.now()
        msg = JointState
        msg.header = header
        msg.name = ['pelvis,shoulder','elbow','wrist']
        msg.position = self.present_current
        self.pub_current.publish(msg)

    # for present position
    def publish_present_position(self):
        header = Header()
        header.stamp = rospy.Time.now()
        msg = JointState
        msg.header = header
        msg.name = ['pelvis,shoulder','elbow','wrist']
        msg.position = self.present_position
        self.pub_pp.publish(msg)

    def on_receive_callback(self,data):
        self.rotations = np.array(data.position)

    ############################## 
    # Actuate functions
    ##############################

    def getGoalPositions(self):
        """ transform displacements of the cable into a rotation of the motor"""
        rot = self.rotations*self.gearRatio
        rot = rot*4095/(2*math.pi)
        rot = self.initPosition+self.orientation*rot
        self.goalPositions = rot.astype(int)
            
    def actuate(self):
        
        while not rospy.is_shutdown():

            # write position
            self.getGoalPositions()
            self.motors.write_position(self.goalPositions)
            
            # read and publish
            self.present_position = self.motors.read_position()
            self.present_current = self.motors.read_current()
            self.publish_current()
            self.publish_present_position()
            self.rosRate.sleep()

        self.motors.turnOFF()


def main():
    # parameters 
    rate = 1/0.01
    rosName = "motors_dynamixel_node"

    motors = ROS_Dynamixel_node(rosName,rate)

    motors.actuate()
    
if __name__=='__main__':
    main()