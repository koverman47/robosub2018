#!/usr/bin/env python

import time, matrix, numpy as np
from scipy import integrate


class Controller():

    def __init__(self, recursion_depth = 1, kp = 1.0, ki = 0, kd = 0.6):
        self.errors = []
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.threshold = 0.001
        self.max_recursion = recursion_depth
        self.transform = [  [1, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 1, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 1, 1, 1],
                            [0, 0, 0, 0, 0.228092, -0.228092, 0.230630, -0.230632],
                            [0, 0, 0, 0, -0.109601, -0.109601, 0.110871, 0.110871],
                            [-0.225552, 0.225552, 0.28956, -0.310642, 0, 0, 0, 0]  ]
        self.inverse = np.linalg.pinv(self.transform).tolist()
        self.clean_inverse() # numpy moore-penrose inverse is bugged


    def clean_inverse(self):
        for r in range(len(self.inverse)):
            for c in range(len(self.inverse[r])):
                self.inverse[r][c] = round(self.inverse[r][c], 15)


    # replace errors below threshold with zero
    def test_threshold(self, vector):
        tested = []
        for v in vector:
            if abs(v) < self.threshold:
                tested.append(0)
            else:
                tested.append(v)
        
        return tested


    # Calculate PID and return transformation (desired forces -> motor commands)
    def get_motor_commands(self, estimated, destination):
        self.errors.append(self.test_threshold(matrix.subtract_vector(destination, estimated)))

        ed = self.get_derivative()
        if self.ki != 0:
            ei = [0 for i in range(len(ed))]
        else:
            ei = self.get_integral()

        desired = self.pid(self.errors[-1], ei, ed)

        return self.calc_motor_commands(desired)


    # Transforms 6d forces vector into 8d motor commands vector
    def calc_motor_commands(self, desired):
        commands = []

        for r in range(len(self.inverse)):
            sigma = 0
            for c in range(len(self.inverse[r])):
                sigma += desired[c] * self.inverse[r][c] # Apply the transformation on the desired forces vector
            commands.append(sigma)

        large = abs(max(commands, key=abs)) # get max magnitude for scaling
        for c in range(len(commands)):
            if large != 0:
                commands[c] = commands[c] / large # scaled down to [-1, 1]
            else:
                commands[c] = 0 # don't divide by zero

        return commands



    def get_derivative(self):
        derivative = matrix.recursive_vector_sum(self.errors, min(self.max_recursion, len(self.errors)))
        time = 1.0 / min(self.max_recursion, len(self.errors))

        for d in range(len(derivative)):
            if self.errors[-1][d] == 0:
                derivative[d] = 0

        return matrix.constant_multiply_vector(derivative, time)


    def get_integral(self):
        if len(self.errors) < self.max_recursion:
            return [0 for k in range(6)]

        data = [0 for m in range(self.max_recursion)]
        integral = []
        for i in range(len(self.errors[-1])):
            for j in range(self.max_recursion):
                data[j] = self.errors[-j][i]
            integral.append(integrate.trapz(data))

        for i in range(len(integral)):
            if self.errors[-1][i] == 0:
                integral[i] = 0

        return integral


    def pid(self, ep, ei, ed):
        p = matrix.constant_multiply_vector(ep, self.kp)
        i = matrix.constant_multiply_vector(ei, self.ki)
        d = matrix.constant_multiply_vector(ed, self.kd)
        return matrix.add_vector(p, matrix.add_vector(i, d))


    








        
