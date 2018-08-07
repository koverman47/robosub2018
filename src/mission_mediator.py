#!/usr/bin/env python

import rospy
from lib.mission import Mission
from robosub2018.msg import State, ObjectDetection, Arm


'''
Maintain Depth: Make sure all desired states have z-coordinate greater than one
'''

detection_msg = None
current_msg = None
desired_msg = None
desired_publisher = None


def mediate():
    global detection_msg

    desired_publisher = rospy.Publisher('borbcat/desired', State, queue_size=1)
    rospy.init_node('captain')

    rospy.Subscriber('vision/detection', ObjectDetection, detection_callback)
    rospy.Subscriber('borbcat/state', State, current_state_callback)

    desired_msg = State()
    desired_msg.header.seq = 0
    desired_msg.header.timestamp = rospy.get_rostime()

    mission = Mission()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        desired_msg.state = mission.get_next_state(detection_msg, current_msg.state)
        desired_msg.header.seq += 1
        desired_msg.header.timestamp = rospy.get_rostime()

        desired_publisher.publish(desired_msg)
        rate.sleep()


def detection_callback(msg):
    global detection_msg
    detection_msg = msg


def current_state_callback(msg):
    global current_msg
    current_msg = msg

if __name__ == "__main__":
    mediate()
