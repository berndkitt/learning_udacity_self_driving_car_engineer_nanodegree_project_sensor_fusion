# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        F = np.eye(params.dim_state)
        
        F[0, 3] = params.dt
        F[1, 4] = params.dt
        F[2, 5] = params.dt

        return F
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        Q = np.zeros((params.dim_state, params.dim_state))
        
        dt = params.dt
        q_x = params.q
        q_y = params.q
        q_z = params.q # RMSE can be improved from 0.32 to 0.31 by using q_z = 0.1
        
        q_x_3 = 1.0 / 3.0 * dt**3 * q_x
        q_x_2 = 0.5 * dt**2 * q_x
        q_x_1 = dt * q_x
        
        q_y_3 = 1.0 / 3.0 * dt**3 * q_y
        q_y_2 = 0.5 * dt**2 * q_y
        q_y_1 = dt * q_y
        
        q_z_3 = 1.0 / 3.0 * dt**3 * q_z
        q_z_2 = 0.5 * dt**2 * q_z
        q_z_1 = dt * q_z
        
        Q[0, 0] = q_x_3
        Q[0, 3] = q_x_2
        Q[1, 1] = q_y_3
        Q[1, 4] = q_y_2
        Q[2, 2] = q_z_3
        Q[2, 5] = q_y_2
        Q[3, 0] = q_x_2
        Q[3, 3] = q_x_1
        Q[4, 1] = q_y_2
        Q[4, 4] = q_y_1
        Q[5, 2] = q_z_2
        Q[5, 5] = q_z_1

        return Q
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        track.x = self.F() * track.x
        track.P = self.F() * track.P * self.F().transpose() + self.Q()
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        x = track.x
        
        K = track.P * meas.sensor.get_H(x).transpose() * np.linalg.inv(self.S(track, meas, meas.sensor.get_H(x)))
        
        track.x = track.x + K * self.gamma(track, meas)
        track.P = (np.eye(params.dim_state) - K * meas.sensor.get_H(x)) * track.P
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        gamma = meas.z - meas.sensor.get_hx(track.x)

        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        S = H * track.P * H.transpose() + meas.R

        return S
        
        ############
        # END student code
        ############
