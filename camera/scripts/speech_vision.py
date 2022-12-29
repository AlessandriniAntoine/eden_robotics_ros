#!/usr/bin/env python3

import time

import rospy
from std_msgs.msg import String

import cv2
from tracking.tracker import Tracker
from tracking.parameters import * 

class Speech_Vision_Node:
    """Node for real time speech recognition with webcam
    """

    def __init__(self,rosName="speech_vision_node"):
        
        # Init ROS node
        rospy.init_node(rosName, anonymous=True)
        try:
            rate = rospy.get_param('/rate')
        except Exception:
            rate = 100
        self.rosRate = rospy.Rate(rate)

        # init publisher
        self.initGraph()
        
        # init camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit() 
        print('Camera open')

        # init variables
        self.prev_frame_time = 0
        self.tracking = False
        self.text = None

        # init parameters
        self.dimension = dimension

        # init tracker
        self.tracker = Tracker(type = tracker_type)

    ############################################ ROS Functions ###########################################

    def initGraph(self):
        self.sub = rospy.Subscriber('/Speech/text',String,self.on_receive_callback,queue_size=10)

    def on_receive_callback(self,data):
        self.text = str(data.data)

    ############################################ Camera Functions ###########################################

    def get_fps(self):
        new_frame_time = time.time()
        fps = round(1/(new_frame_time-self.prev_frame_time),2)
        self.prev_frame_time = new_frame_time
        return fps

    def close_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()

    ############################################ Update ###########################################

    def update(self):
        while self.cap.isOpened() and not rospy.is_shutdown():
            flag, frame = self.cap.read()
            if not flag :
                break
            frame = cv2.resize(frame,self.dimension)
            fps = self.get_fps()
            cv2.putText(frame, f'FPS : {fps}Hz, Command : {self.text}, Tracking : {self.tracking}', (2, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 1, cv2.LINE_AA)

            if self.text == 'track' and not self.tracking : 
                self.tracker.init_tracker(frame)
                if self.tracker.flag:
                    self.tracking = True
                else :
                    print('Error while initializing tracker !')
            if self.text  == 'stop':
                self.tracking = False
            if self.text == 'close':
                break

            if self.tracking : 
                self.tracker.track(frame)
                if not self.tracker.flag :
                    self.tracking = False
            
            cv2.imshow('Frame', frame)
            cv2.waitKey(1)

            self.rosRate.sleep() 
        self.close_camera()

def main():
    vision = Speech_Vision_Node()
    vision.update()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass