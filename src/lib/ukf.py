#!/usr/bin/env python

import math, rospy, time, numpy as np
from sigma_point_set import SigmaPTS as SPT
from robosub2018.msg import State, Depth
from std_msgs.msg import Imu


class UKF():

    def __init__(self, dim, Q, R, G, H, dt):
        self.x = [] # state vector
        self.p = [] # state covariance matrix
        self.Q = [] # motion noise
        self.R = [] # observation noise
        self.G = G
        self.H = H
        self.dt = dt
        self.sigma_pts = SPT(dim, 1, 0.5, 2) # Need to parameters


    # zeta should come preprocessed
    def get_estimate(self, zeta):
        self.sigma_pts.calc_sigma_pts(self.x, self.p) 

        # TODO: Propagate self.sigma_pts.chi
        mu_bar = self.calc_mu_estimate()
        cov_bar = self.calc_cov_estimate(mu_bar)

        self.sigma_pts.calc_sigma_pts(self.mu_bar, self.cov_bar)

        prop = self.calc_g() # prop - big_zeta_bar (check notes)
        zeta_bar = self.calc_zeta_estimate(prop)

        st = self.calc_s(prop, zeta_bar) # check on zeta instead of zeta bar

        cov_xz = self.calc_cov_xz(mu_bar, prop, zeta_bar)

        kappa = self.calc_kalman_gain(cov_bar, st)

        self.x = self.calc_final_mu(mu_bar, kappa, zeta, zeta_bar)
        self.p = self.calc_final_cov(cov_bar, kappa, st)


    def get_state(self, zeta):
        pass


    def calc_mu_estimate(self):
        mu = []
        for i in range(len(self.sigma_pts.n)):
            mu.append(matrix.inner_product(self.sigma_pts.state_weights, self.sigma_pts.chi[i]))
        return mu

    
    def calc_cov_estimate(self, mu_bar):
        diff = []
        #cov = [[0 for i in range(len(mu_bar))] for j in range(len(mu_bar))]
        cov = []
        for i in range(len(self.sigma_pts[0])):
            diff.append([])
            for j in range(len(mu_bar)):
                diff[i].append(self.sigma_pts.chi[j][i] - mu_bar[j])
            out = matrix.constant_matrix_multiply(matrix.outer_product(diff[i], diff[i]), self.sigma_pts.cov_weights[i])
            cov = matrix.add_matrix(cov, add)
        return matrix.add_matrix(cov, self.Q)

    
    # Assuming Identity at the moment
    # TODO: Use propagating functions, G, H
    def calc_propagation(self):
        pass


    def calc_zeta_estimate(self, prop):
        pass
    
    
    def calc_s(self, prop, zeta):
        pass


    def calc_cov_xz(self, mu_bar, prop, zeta_bar):
        pass


    def calc_kalman_gain(self, cov_bar, st):
        pass


    def calc_final_mu(self, mu_bar, kappa, zeta, zeta_bar):
        pass


    def calc_final_cov(self, cov_bar, kappa, st):
        pass

