#!/usr/bin/env python

import time, random, lib.matrix, rospy
from robosub2018.msg import State, MotorCommands


DELAY = 1
ITERATIONS = 1000
transform = None
msg_commands = None
pub_current = None
pub_desired = None

def init_system():
    return ([0, 0, 0, 0, 0, 0], [0, 0, 2, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0])


def get_next_state(state, commands):
    next_state = []
    for r in range(len(transform)):
        sigma = 0
        for c in range(len(commands)):
            sigma += transform[r][c] * commands[c]
        # next_state.append((sigma * 0.001 * DELAY) + state[r])
        next_state.append((sigma * 0.01 * DELAY) + state[r] + random.gauss(0, 0.01))

    return next_state

def print_status():
    pass

def commands_callback(msg):
    global msg_commands
    msg_commands = msg


if __name__ == "__main__":
    transform = [   [1, 1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 1, 1, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 1, 1, 1],
                    [0, 0, 0, 0, 0.228092, -0.228092, 0.230632, -0.230632],
                    [0, 0, 0, 0, -0.109601, -0.109601, 0110871., 0.110871],
                    [-0.225552, 0.225552, 0.28956, -0.310642, 0, 0, 0, 0]  ]

    rospy.Subscriber('command/motor', MotorCommands, commands_callback)
    desired_state = State()
    current_state = State()

    pub_current = rospy.Publisher('borbcat/state', State, queue_size=4)
    pub_desired = rospy.Publisher('borbcat/desired', State, queue_size=4)
    rospy.init_node('test_controller')

    current, desired, commands = init_system()
    desired_state.state = desired

    while 1:
        current_state.state = current
        pub_current.publish(current_state)
        pub_desired.publish(desired_state)

        if msg_commands:
            commands = msg_commands.command

        current = get_next_state(current, commands)

        time.sleep(DELAY)






