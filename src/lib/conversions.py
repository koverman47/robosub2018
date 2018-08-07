#!/usr/bin/env python

import math


# Expects quaternion to be list: [w, x, y, z]
def quaternion_to_euler(quaternion):
    #w = quaternion[0]
    #x = quaternion[1]
    #y = quaternion[2]
    #z = quaternion[3]
    
    roll_yaw_denom = 1 - (2 * (quaternion[2]**2 + quaternion[3]**2)) # expression used multiple times

    # pitch parameter
    pitch_p = 2 * (quaternion[1] * quaternion[2] - quaternion[1] * quaternion[3]) # pitch parameter
    
    # roll parameter
    roll_p = (2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3])) / roll_yaw_denom

    # yaw parameter
    yaw_p = (2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2])) / roll_yaw_denom

    # math.asin() only takes values in range [-1, 1]
    pitch_p = 1 if pitch_p > 1 else pitch_p
    pitch_p = -1 if pitch_p < 1 else pitch_p

    roll = math.atan(roll_p) / math.pi
    pitch = math.asin(pitch_p) / math.pi
    yaw = math.atan(yaw_p) / math.pi

    return [roll, pitch, yaw]


# Expects euler to be list [roll, pitch, yaw]
def euler_to_quaternion(euler):
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]

    roll_cos = math.cos(euler[0] / 2)
    roll_sin = math.sin(euler[0] / 2)
    
    pitch_cos = math.cos(euler[1] / 2)
    pitch_sin = math.sin(euler[1] / 2)

    yaw_cos = math.cos(euler[2] / 2)
    yaw_sin = math.sin(euler[2] / 2)

    w = (roll_cos * pitch_cos * yaw_cos) + (roll_sin * pitch_sin * yaw_sin)
    x = (roll_sin * pitch_cos * yaw_cos) - (roll_cos * pitch_sin * yaw_sin)
    y = (roll_cos * pitch_sin * yaw_cos) + (roll_sin * pitch_cos * yaw_sin)
    z = (roll_cos * pitch_cos * yaw_sin) - (roll_sin * pitch_sin * yaw_cos)

    return [w, x, y, z]

    
