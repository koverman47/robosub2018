#!/usr/bin/env python

import rospy
from robosub2018.msg import State
from sensor_msgs.msg import Imu


current_msg = None
desired_msg = None
desired_publisher = None
current_publisher = None

def mission():
    
    desired_publisher = rospy.Publisher('borbcat/desired', State, queue_size=1)
    current_publisher = rospy.Publisher('borbcat/state', State, queue_size=1)
    rospy.Subscriber('sensors/imu', Imu, imu_callback)
    rospy.Subcriber('sensors/depth', Depth, depth_callback)
    
    desired_msg = State()
    desired_msg.header.seq = 0
    desired_msg.header.timestamp = rospy.get_rostime()
    
    current_msg = State()
    current_msg.header.seq = 0
    current_msg.header.timestamp = rospy.get_rostime()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        desired_msg.state = [0, 0, 0, 0, 0, 0]
        desired_msg.header.seq += 1
        desired_msg.header.timestamp = rospy.get_rostime()
        
        desired_publisher.publish(desired_msg)
        current_publisher.publish(current_msg)
        rate.sleep()
        
def imu_callback(msg):
    global current_msg
s   current_msg.state[1] = msg.state[1]
    
    
def depth_callback(msg):
    global current_msg
    current_msg.state[2] = msg.depth
    current_msg.header.seq += 1
    current_msg.header.timestamp = rospy.get_rostime()
    
        
while __name__ == "__main__":
    mission()
