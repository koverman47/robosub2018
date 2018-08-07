#!/usr/bin/env python

import math
import rospy
import time
import matrix
import numpy as np

from sigma_point_set import SigmaPTS as SPT
from robosub2018.msg import State
from robosub2018.msg import Depth
from std_msgs.msg import Imu


class UKF():

    def __init__(self, dim, R, Q, G, H):
        self.x = []  # state mean belief
        self.P = []  # state covariance belief
        self.Q = Q   # state transition covariance
        self.R = R   # measurement covariance noise
        self.G = G   # state transition function
        self.H = H   # measurement transition function
        
        self.lamb = 1
        self.alpha = 0.5
        self.beta = 2


    def get_belief(self, state_t_1, covariance_t_1, control, measurement):
        chi_t_1 = SPT()
        chi_t_1.calc_sigma_pts(state_t_1, covariance_t_1, self.alpha, self.beta, self.lamb)
        chi_t_1.transform(self.G)

        mu_bar, cov_bar = chi_t_1.reconstruct()

        chi_bar = SPT()
        chi_bar.calc_sigma_pts(mu_bar, cov_bar, self.alpha, self.beta, self.lamb)

        zeta_bar = SPT() #TODO: Need deep copy of chi_bar
        zeta_bar.transform(self.H)
        
        zeta_hat, zeta_hat_cov = zeta_bar.reconstruct()

        cross_cov = self.get_cross_covariance()

        gain = self.get_kalman_gain()

        mu = self.get_mean_correction()
        cov = self.get_covariance_correction()
        return (mu, cov)


    def get_cross_covariance(self, spt1, mean1, spt2, mean2):
        pass


    def get_kalman_gain(self, cross_cov, zeta_cov):
        inv = np.linalg.pinv(zeta_cov).tolist()
        return matrix.matrix_multiply(cross_cov, inv)


    def get_mean_correction(self, mu_bar, gain, measurement, zeta_mean):
        return add_matrix(mu_bar, matrix_multiplication(gain, matrix.subtract_vector(measurement, zeta_mean)))


    def get_covariance_correction(self, cov_bar, gain, zeta_cov):
        pass # Need transpose







