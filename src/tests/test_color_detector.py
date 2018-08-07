#!/usr/bin/env python

import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robosub2018.msg import State


class TestColorDetector:

    def __init__(self):
        rospy.init_node("test_color_detector")

        self._pub_image = rospy.Publisher("sensors/camera_forward/image_raw", Image, queue_size=10)
        self._pub_state = rospy.Publisher("state", State, queue_size=10)

        state_msg = State()
        for i in range(len(state_msg.state)):
            state_msg.state[i] = 0

        cv_bridge = CvBridge()
        image = cv2.imread("161.jpg", cv2.IMREAD_UNCHANGED)
        #cv2.imshow('image', image)
        #cv2.waitKey(0)
        image_msg = cv_bridge.cv2_to_imgmsg(image, "bgr8")
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._pub_image.publish(image_msg)
            self._pub_state.publish(state_msg)
            rate.sleep()





if __name__ == "__main__":
    TestColorDetector()
