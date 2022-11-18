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
        dt = params.dt
        matrix_F = np.matrix(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )

        return matrix_F
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        dt = params.dt
        q = params.q
        qt = params.q * dt
        qtt = 1/2 * dt **2 * q
        qttt = 1/3 * dt ** 3 * q

        q1 = qt 
        q2 = qtt
        q3 = qttt

        matrix_Q = np.matrix(
            [
                [q3, 0, 0, q2, 0, 0],
                [0, q3, 0, 0, q2, 0],
                [0, 0, q3, 0, 0, q2],
                [q2, 0, 0, q1, 0, 0],
                [0, q2, 0, 0, q1, 0],
                [0, 0, q2, 0, 0, q1],
            ]
        )
        return matrix_Q
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        x = F*track.x
        P = F*track.P*F.transpose() + self.Q()
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        x = track.x
        P = track.P
        R = meas.R 
        gamma = self.gamma(track, meas)
        H = meas.sensor.get_H(x)
        S = self.S(track, meas, H)

        K = P*H.T*np.linalg.inv(S)
        X = x + K*gamma
        I = np.identity(params.dim_state)
        P = (I - K*H) * P
        track.set_x(X)
        track.set_P(P)

        track.update_attributes(meas)

        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        x = track.x 
        gamma =meas.z - meas.sensor.get_hx(x)
        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        R = meas.R 
        P = track.P  
        S = H*P*H.T + R

        return S
        
        ############
        # END student code
        ############ 