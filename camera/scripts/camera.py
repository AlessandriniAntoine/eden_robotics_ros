#!/usr/bin/env python3

import cv2
import rospy

class Camera_Node:
    """
    Node to display webcam window
    """

    def __init__(self,rosName="camera_node",rate=100):

        # Init ROS2 node
        rospy.init_node(rosName, anonymous=True)
        self.rosRate = rospy.Rate(rate)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit() 
        print('Camera open')

        
    def display(self):
        while True and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            # Display the resulting frame
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == ord('q'):
                break
            self.rosRate.sleep()


    def close_camera(self):
        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    camera = Camera_Node()
    camera.display()
    camera.close_camera()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass