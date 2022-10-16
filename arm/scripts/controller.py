#!/usr/bin/env python3

import time
import rospy


from geometry_msgs.msg import Point,PoseStamped

from kinematics.parameters import *


class CloseLoop_Controller_Node:
    """The goal of this controller it to :
        - add a gain
        - add an integrator
        - add an antiwindup
    """

    def __init__(self,rosName="closedLoop_controller_node"):


        # Init ROS node
        rospy.init_node(rosName, anonymous=True)
        try:
            rate = rospy.get_param('/rate')
        except :
            rate = 100
        self.rosRate = rospy.Rate(rate)
        self.init_graph()

        # parameters for the Proportionnal controller
        self.time = time.time()

        # controller parameters:
        self.ki = ki
        self.kp = kp
        self.sat = sat
        self.kb = kb

        # init variables
        self.reference = qe
        self.command = qe
        self.command_sat = qe
        self.measure = qe

        self.integrator_term = qe

    #######################################
    # ROS function
    #######################################

    def init_graph(self):
        self.sub_ref = rospy.Subscriber('/Robot/ref/position',Point,self.on_receive_callback_ref,queue_size=10)
        self.sub_mea = rospy.Subscriber('/Robot/state/measure',PoseStamped,self.on_receive_callback_mea,queue_size=10)
        self.pub_com = rospy.Publisher('/Robot/ref/command',Point,queue_size=10)

    def publish(self):
        msg = Point()
        msg.x = self.command_sat[0]
        msg.y = self.command_sat[1]
        msg.z = self.command_sat[2]
        self.pub_com.publish(msg)
    
    def on_receive_callback_ref(self,data):
        self.reference = np.array([data.x,data.y,data.z])

    def on_receive_callback_mea(self,data):
        self.measure = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])

    ########################################
    # Proportionnal controller functions
    ########################################

    def controller(self):

        epsilon = self.reference - self.measure

        proportionnal_term = self.kp*epsilon
        ki_eps = self.ki*epsilon
        
        windup_error = self.kb*(self.command_sat-self.command)

        self.integrator_term = self.integrator_term + (ki_eps+windup_error)*self.dt

        self.command = proportionnal_term + self.integrator_term
        
        for i in range(len(self.command)):
            if self.command[i] >= self.sat:
                self.command_sat[i] = self.sat
            elif self.command[i] <= -self.sat:
                self.command_sat[i] = -self.sat
            else :
                self.command_sat[i] = self.command[i]

    ########################################
    # other functions
    ########################################
    
    def update(self):
        while not rospy.is_shutdown():
            
            # get real time step
            t2 = time.time()
            self.dt = t2-self.time
            self.time = t2

            # get new ouput value
            self.controller()
            self.publish()
            self.rosRate.sleep()


def main():
    controller = CloseLoop_Controller_Node()
    controller.update()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass