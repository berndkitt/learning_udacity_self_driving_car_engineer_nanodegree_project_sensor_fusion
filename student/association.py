# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        # get number of tracks and measurements
        num_tracks = len(track_list) # represents the number of rows of the association matrix
        num_meas = len(meas_list) # represents the number of columns of the association matrix
        
        # the following only works for at most one track and one measurement
        self.association_matrix = np.matrix([]) # reset matrix
        self.unassigned_tracks = [] # reset lists
        self.unassigned_meas = []
        
        if len(meas_list) > 0:
            self.unassigned_meas = np.arange(0, num_meas, 1)
        if len(track_list) > 0:
            self.unassigned_tracks = np.arange(0, num_tracks, 1)
        if len(meas_list) > 0 and len(track_list) > 0: 
            self.association_matrix = np.inf * np.ones((num_tracks, num_meas)) # initialize with infinity
            
            # populate association matrix in case that at least one measurement and one track is available
            for i_track in range(num_tracks):
                for i_meas in range(num_meas):
                    mhd = self.MHD(track_list[i_track], meas_list[i_meas], KF)
                    
                    if self.gating(mhd, meas_list[i_meas].sensor): # update those entries which pass the gating only (others remain infinity)
                        self.association_matrix[i_track, i_meas] = mhd
        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############
        
        # get dimension of the association matrix
        association_matrix_num_rows = self.association_matrix.shape[0]
        association_matrix_num_cols = self.association_matrix.shape[1]
        
        # find index with minimum MHD
        idx_minimum = np.argmin(self.association_matrix)
        
        row_idx_minimum = idx_minimum // association_matrix_num_cols
        col_idx_minimum = idx_minimum - row_idx_minimum * association_matrix_num_cols
        
        # get value of minimum MHD
        value_minimum_distance = self.association_matrix[row_idx_minimum, col_idx_minimum]
        
        # check whether the minimum MHD is a valid value and remove 
        if np.isinf(value_minimum_distance):
            return np.nan, np.nan
        else:
            update_track = self.unassigned_tracks[row_idx_minimum]
            update_meas = self.unassigned_meas[col_idx_minimum]
            
            # remove indices from lists
            self.unassigned_tracks = np.delete(self.unassigned_tracks, row_idx_minimum)
            self.unassigned_meas   = np.delete(self.unassigned_meas, col_idx_minimum)
            
            # remove row and column from association matrix
            self.association_matrix = np.delete(self.association_matrix, row_idx_minimum, 0) # row
            self.association_matrix = np.delete(self.association_matrix, col_idx_minimum, 1) # column
            
        ############
        # END student code
        ############ 
            return update_track, update_meas     

    def gating(self, MHD, sensor): 
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        IsInsideGate = False
        
        # calculate threshold
        threshold = chi2.ppf(params.gating_threshold, sensor.dim_meas)
        
        if MHD < threshold:
            IsInsideGate = True
        
        return IsInsideGate
        
        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############
        # get H matrix from sensor
        H = meas.sensor.get_H(track)
        
        # calculate covariance matrix of the residual
        S = KF.S(track, meas, H)
        
        # calculate residual gamma
        gamma = KF.gamma(track, meas)
        
        # calculate MHD
        mhd = gamma.transpose() * np.linalg.inv(S) * gamma
        
        return mhd
        
        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)