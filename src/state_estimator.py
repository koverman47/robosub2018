#!/usr/bin/env python

import math, rospy, numpy as np
from robosub2018.msg import State, Depth
from sensors_msgs.msg import Imu
from lib.ukf import UKF


depth_msg = None 
imu_msg = None
state_msg = None
state_pub = None


def depth_callback(msg):
    global depth_msg
    depth_msg = msg


def imu_callback(msg):
    global imu_msg
    imu_msg = msg


def estimate():
    rospy.Subscriber('sensors/depth', Depth, depth_callback)
    rospy.Subscriber('sensors/imu', Imu, imu_callback)
    state_pub = rospy.Publisher('borbcat/state', State, queue_size=8)

    state_msg = State()
    state_msg.header.seq = 0
    state_msg.stamp = rospy.get_rostime()

    orientation = UKF()
    # transform acceleration readings with respect to orientation
    acceleration = UKF() # Don't forget Parameters

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rotation = orientation.get_estimate(imu_msg)
        state_msg.state = acceleration.get_state_estimate(imu_msg, depth_msg)
        state_msg.header.seq += 1
        state_msg.header.stamp = rospy.get_rostime()

        state_pub.publish(state_msg)



if __name__ == "__main__":
    estimate()
