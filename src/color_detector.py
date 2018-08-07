#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robosub2018.msg import ObjectDetection, State


class ColorObjectDetector:
    
    def __init__(self):
        rospy.init_node('color_detector')

        #TODO: correct channel names
        rospy.Subscriber("sensors/camera_forward/image_raw", Image, self.receive_image)
        rospy.Subscriber("borbcat/state", State, self.receive_state)

        # Channels
        self._image_pub = rospy.Publisher("vision/image", Image, queue_size=10)
        self._object_pub = rospy.Publisher("vision/detection", ObjectDetection, queue_size=10)


        # Misc
        self._last_state = None
        self._search_object = QualPole() # Object we're expecting to see
        self._object_msg = ObjectDetection()

        # TODO: you need to calculate the camera's focal length by setting an object with known width a measured distance away from the camera
        # See here: https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
        _PIXEL_WIDTH = 100
        _KNOWN_DISTANCE = 1 # meters
        _KNOWN_WIDTH = .15 # meters
        self._focal_length = (_PIXEL_WIDTH * _KNOWN_DISTANCE) / _KNOWN_WIDTH

        # Image processing parameters
        self._cv_bridge = CvBridge()

        self._target_color = (175, 210, 119)
        self._thresh_size = 20
        self._min_thresh = (self._target_color[0]-self._thresh_size, self._target_color[1]-self._thresh_size, self._target_color[2]-self._thresh_size)
        self._max_thresh = (self._target_color[0]+self._thresh_size, self._target_color[1]+self._thresh_size, self._target_color[2]+self._thresh_size)

        self._kernel = np.ones((5,5), np.uint8)

        rospy.spin()

    def receive_image(self, image_msg):
        if self._last_state is not None:
            image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.color_object_detect(image)

    def receive_state(self, state):
        self._last_state = state

    def color_object_detect(self, image):
        image = self.preprocess_image(image)
        image = self.color_threshold(image)
        image = self.erode_dilate(image)
        contour = self.get_contours(image)
        
        coords = self.get_object_coords()

        image = self.draw_debug(image, contour)
                
        self.publish_image(image) 
        self.publish_object(coords)

    def show_image(self, image):
        cv2.imshow('test', image)
        cv2.waitKey(10)

    def preprocess_image(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return hsv

    def color_threshold(self, image):
        thresh = cv2.inRange(image, self._min_thresh, self._max_thresh)
        return thresh

    def erode_dilate(self, image):
        image = cv2.erode(image, self._kernel, iterations=1)
        image = cv2.dilate(image, self._kernel, iterations=1)
        return image

    def get_contours(self, image):
        im2, contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        longest = max(contours, key=len)
        return longest

    def get_object_distance(self, contour):
        _, _, pixel_width, _ = cv2.boundingRect(contour)
        return (self._object.width * self._focal_length) / pixel_width

    def get_object_coords(self):
        # The object and the camera form a right triangle, where a line drawn from the camera to the center of the object forms the hypotenuse.
        # Solving for the rest of the triangle gives us the coordinates relative to the sub, which can then be translated to world space using sub state
        # TODO
        return (0, 0, 0)

    def draw_debug(self, image, contour):
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        image = cv2.drawContours(image, [contour], -1, (0,255,0), 2)
        return image


    def publish_image(self, image):
        img_msg = self._cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self._image_pub.publish(img_msg)


    def publish_object(self, object_data):
        self._object_msg.header.seq += 1
        self._object_msg.header.stamp = rospy.get_rostime()
        self._object_msg.x = object_data[0]
        self._object_msg.y = object_data[1]
        self._object_msg.z = object_data[2]
        self._object_pub.publish(self._object_msg)


class QualPole:
    # Object width in meters
    width = .15

if __name__ == "__main__":
    ColorObjectDetector()
