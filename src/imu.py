#!/usr/bin/env python

import serial, rospy, re, math
import numpy as np
from sensor_msgs.msg import Imu


port = '/dev/ttyUSB0'
baud = 115200

def imu():
    global ser
    publisher = rospy.Publisher('sensors/imu' Imu, queue_size=10)
    ser = None

    rospy.init_node('imu')
    rate = rospy.Rate(100)

    imu_msg = Imu()
    imu_msg.header.seq = 0
    imu_msg.header.frame_id = "imu0"
    imu_msg.orientation_covariance = [0.000001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0, 0.000001]
    imu_msg.angular_velocity_covariance = [0.000001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0, 0.000001]
    imu_msg.linear_acceleration_covariance = [0.00117, 0.0, 0.0, 0.0, 0.00277, 0.0, 0.0, 0.0, 0.00034]

    try:
        ser = serial.Serial(port, baud)
        ser.flush()
    except serial.SerialException:
        rospy.logerr("Error Connceting to IMU")

    while not rospy.is_shutdown():
        if not ser:
            continue

        imu_msg.header.seq += 1
        imu_msg.header.stamp = rospy.get_rostime()

        # Magnetometer
        # Conversions: None
        data = get_imu_data("$PSPA,QUAT\r\n")
        imu_msg.orientation.w = data[0]
        imu_msg.orientation.x = data[1]
        imu_msg.orientation.y = data[2]
        imu_msg.orientation.z = data[3]

        # Gyrometer
        # Conversions: millidegrees -> radians
        data = get_imu_data("$PSPA,G\r\n")
        imu_msg.angular_velocity.x = data[0] * math.pi / 180 / 1000
        imu_msg.angular_velocity.y = data[1] * math.pi / 180 / 1000
        imu_msg.angular_velocity.z = data[2] * math.pi / 180 / 1000

        # Accelerometer
        # Conversions: milli-g's -> m/s^2
        data = get_imu_data("$PSPA,A\r\n")
        imu_msg.linear_acceleration.x = data[0] * 9.80665 / 1000
        imu_msg.linear_acceleration.y = data[1] * 9.80665 / 1000
        imu_msg.linear_acceleration.z = data[2] * 9.80665 / 1000

        publisher.publish(imu_msg)
        rate.sleep()


def get_imu_data(self, command):
    self.ser.write(command)
    data = ser.readline()
    values = np.array(re.findall('([-\d.]+)', data)).astype(np.float)
    return values

if __name__ == '__main__':
    try:
        imu()
    except rospy.ROSInterruptException:
        pass
