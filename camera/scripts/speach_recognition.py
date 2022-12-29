#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from speech.recognizer import SpeechRecognition

class Speech_Recognition_Node:
    """Node for real time speech recognition
    """

    def __init__(self,rosName="speech_recognition_node"):
        
        # Init ROS node
        rospy.init_node(rosName, anonymous=True)
        try:
            rate = rospy.get_param('/rate')
        except Exception:
            rate = 100
        self.rosRate = rospy.Rate(rate)

        # init publisher
        self.initGraph()
        
        # init speech recognition
        self.sr = SpeechRecognition()

    def initGraph(self):
        self.pub = rospy.Publisher('/Speech/text',String,queue_size=10)

    def publish(self):
        msg = String()
        msg.data = self.sr.text
        self.pub.publish(msg)

    def update(self):
        while not rospy.is_shutdown():
            self.sr.predict_mic()
            self.publish()
            if self.sr.text == "close":
                self.sr.terminate()
                break
            self.rosRate.sleep() 

def main():
    speech = Speech_Recognition_Node()
    speech.update()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass