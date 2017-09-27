#!/usr/bin/env python
# Ransalu Senanayake, Fabio Ramos 2017


import numpy as np
import copy
import matplotlib.pylab as pl
import pandas as pd
from sklearn.metrics.pairwise import rbf_kernel
import sys
import time as comp_timer
import os


from sklearn.linear_model.base import LinearClassifierMixin, BaseEstimator
from scipy.special import expit
from scipy.linalg import solve_triangular

import rospy
import rospkg
import nav_msgs.msg
#from nav_msgs.msg import OccupancyGrid
#from nav_msgs.msg import MapMetaDeta

class BayesianHilbertMap(LinearClassifierMixin, BaseEstimator):
    def __init__(self, gamma=0.075*0.814, grid=None, cell_resolution=(5, 5), cell_max_min=None, X=None, calc_loss=False):
        """
        :param gamma: RBF bandwidth
        :param grid: if there are prespecified locations to hinge the RBF
        :param cell_resolution: if 'grid' is 'None', resolution to hinge RBFs
        :param cell_max_min: if 'grid' is 'None', realm of the RBF field
        :param X: a sample of lidar locations to use when both 'grid' and 'cell_max_min' are 'None'
        """
        self.gamma = gamma
        if grid is not None:
            self.grid = grid
        else:
            self.grid = self.__calc_grid_auto(cell_resolution, cell_max_min, X)
        self.calc_loss = calc_loss
        self.intercept_, self.coef_, self.sigma_ = [0], [0], [0]
        self.scan_no = 0

    def __calc_grid_auto(self, cell_resolution, max_min, X):
        """
        :param X: a sample of lidar locations
        :param cell_resolution: resolution to hinge RBFs as (x_resolution, y_resolution)
        :param max_min: realm of the RBF field as (x_min, x_max, y_min, y_max)
        :return: numpy array of size (# of RNFs, 2) with grid locations
        """

        if max_min is None:
            # if 'max_min' is not given, make a boundarary based on X
            # assume 'X' contains samples from the entire area
            expansion_coef = 1.2
            x_min, x_max = expansion_coef*X[:, 0].min(), expansion_coef*X[:, 0].max()
            y_min, y_max = expansion_coef*X[:, 1].min(), expansion_coef*X[:, 1].max()
        else:
            x_min, x_max = max_min[0], max_min[1]
            y_min, y_max = max_min[2], max_min[3]

        xx, yy = np.meshgrid(np.arange(x_min, x_max, cell_resolution[0]), \
                             np.arange(y_min, y_max, cell_resolution[1]))
        grid = np.hstack((xx.ravel()[:, np.newaxis], yy.ravel()[:, np.newaxis]))

        return grid

    def __sparse_features(self, X):
        """
        :param X: inputs of size (N,2)
        :return: hinged features with intercept of size (N, # of features + 1)
        """
        rbf_features = rbf_kernel(X, self.grid, gamma=self.gamma)
        return np.hstack((np.ones(X.shape[0])[:, np.newaxis], rbf_features))

    def __lambda(self, epsilon):
        """
        :param epsilon: epsilon value for each data point
        :return: local approximation
        """
        return 0.5/epsilon*(expit(epsilon)-0.5)

    def __calc_loss(self, X, mu0, Sig0_inv, mu, Sig_inv, Ri, epsilon):
        Sig = np.dot(Ri.T, Ri)
        R0 = np.linalg.cholesky(Sig0_inv)
        R0i = solve_triangular(R0, np.eye(X.shape[1]), lower=True)
        S0 = np.dot(R0i.T, R0i)
        loss = 0.5 * np.linalg.slogdet(Sig)[1] - 0.5 * np.linalg.slogdet(S0)[1] + 0.5 * mu.T.dot(Sig_inv.dot(mu)) - 0.5 * mu0.T.dot(Sig0_inv.dot(mu0))
        loss += (np.sum(np.log(expit(epsilon)) - 0.5 * epsilon + self.__lambda(epsilon) * epsilon ** 2))

        return loss

    def __calc_posterior(self, X, y, epsilon, mu0, Sig0_inv, full_covar=False):

        lam = self.__lambda(epsilon)

        Sig_inv = Sig0_inv + 2 * np.dot(X.T*lam, X)

        m_right = Sig0_inv.dot(mu0) + np.dot(X.T, (y - 0.5))
        L_of_Sig_inv = np.linalg.cholesky(Sig_inv)
        Z = solve_triangular(L_of_Sig_inv, m_right, lower=True)
        mu = solve_triangular(L_of_Sig_inv.T, Z, lower=False)

        L_inv_of_Sig_inv = solve_triangular(L_of_Sig_inv, np.eye(X.shape[1]), lower=True)

        if full_covar:
            Sig = np.dot(L_inv_of_Sig_inv.T, L_inv_of_Sig_inv)
            return mu, Sig
        else:
            return mu, Sig_inv, L_inv_of_Sig_inv

    def fit(self, X, y):

        # If first run, set m0, S0i (this is optional, but emperically good enough rather than looking at the convergence)
        if self.scan_no == 0:
            self.mu = np.zeros((self.grid.shape[0] + 1))
            self.Sig_inv = 0.0001 * np.eye((self.grid.shape[0] + 1)) #0.01 for sim, 0
            self.n_iter = 10
        else:
            self.n_iter = 1

        epsilon = 1
        X_orig = copy.copy(X)

        for i in range(self.n_iter):
            X = self.__sparse_features(X_orig)

            # E-step: update Q(w)
            self.mu, self.Sig_inv, self.Ri = self.__calc_posterior(X, y, epsilon, self.mu, self.Sig_inv)

            # M-step: update epsilon
            XMX = np.dot(X, self.mu)**2
            XSX = np.sum(np.dot(X, self.Ri.T) ** 2, axis=1)
            epsilon = np.sqrt(XMX + XSX)

            # Calculate loss, if specified
            if self.calc_loss is True:
                print("  scan={}, iter={} => loss={:.1f}".format(self.scan_no, i,
                      self.__calc_loss(X, np.zeros((self.grid.shape[0] + 1)), 0.01*np.eye((self.grid.shape[0] + 1)),
                        self.mu, self.Sig_inv, self.Ri, epsilon)))

        self.intercept_ = [0]
        coef_, sigma_ = self.__calc_posterior(X, y, epsilon, self.mu, self.Sig_inv, True)

        self.intercept_ = 0
        self.coef_[0] = coef_
        self.sigma_[0] = sigma_
        self.coef_ = np.asarray(self.coef_)
        self.scan_no += 1

    def predict_proba(self, X_q):
        X_q = self.__sparse_features(X_q)#[:, 1:]
        scores = self.decision_function(X_q)
        #X_q = np.hstack((np.ones([X_q.shape[0], 1]), X_q))

        sigma = np.asarray([np.sum(np.dot(X_q, s) * X_q, axis=1) for s in self.sigma_])
        ks = 1. / (1. + np.pi * sigma / 8) ** 0.5
        probs = expit(scores.T * ks).T
        if probs.shape[1] == 1:
            probs = np.hstack([1 - probs, probs])
        else:
            probs /= np.reshape(np.sum(probs, axis=1), (probs.shape[0], 1))
        return probs

def load_parameters(case, fname):
    cell_resolution = (0.5,0.5)
    #cell_max_min = (0, 5.0, -0.45, 3.0)
    cell_max_min = (0, 3.5, 0, 3.5)
    gamma = 0.2
    parameters = \
        {'sim': \
             (fname,
              '',
              '',
              '',
              cell_resolution,
              cell_max_min,
              2,
              0.3,
              gamma
            ),
        }

    return parameters[case]

# Write the hilmap map results to a file that can be loaded into an occupancy grid
# Only way to know the correct locations is to read the locations from the grid used in training
def writeMap(bhm_mdl, fname_read, fname_write):
    print 'In writeMap'
    print 'fname_read: %s' % fname_read
    print 'fname_write: %s' % fname_write

    # Get initial location, cell resolution, and width+height
    # Open file
    f = open(fname_read, 'r')
    lines = f.read().split('\n')

    locs = []
    print len(lines)
    # For each line, get the location
    for i,l in enumerate(lines):
        print i
        print l
        # Sometimes the last entry is empty
        if l != '':
            # Split l by commas
            t = l.split(',')
            if int(t[0]) == 0:
                x = float(t[1])
                y = float(t[2])
                locs.append([x, y])
    locs_array = np.array(locs)
        
    p = bhm_mdl.predict_proba(locs_array)
    gamma = bhm_mdl.gamma

    # Open file for writing
    f = open(fname_write, 'w') 

    for i,l in enumerate(locs_array):
        # Make a string (x,y,prob)
        s = "%f,%f,%f,%f" % (l[0], l[1], p[i][1], gamma)
        
        # Write to file
        f.write(s + os.linesep)
        
    print 'Exiting writeMap'

def main_bhm():

    # Setup paths
    rospack = rospkg.RosPack()
    ros_pkg_path = rospack.get_path('ramp_sensing')
    data_dir_str = 'data'
    occ_map_data_dir_str = 'occ_map_data'
    dir_combined = os.path.join(ros_pkg_path, data_dir_str, occ_map_data_dir_str)
    
    print('ros_pkg_path: %s\ndata_dir_str: %s\nocc_map_data_dir_str: %s\ndir_combined: %s' % \
            (ros_pkg_path, data_dir_str, occ_map_data_dir_str, dir_combined))

    # Get files from data directory
    files = \
    os.listdir(dir_combined)
    print files

    #for fname in files:

        #p = os.path.join('occ_map_data', fname)
        
    p = os.path.join(dir_combined, 'all.csv')
    print('Calling load_parameters')
    [fn_train, fn_test, fn_test_occ_noocc, fn_test_occ, cell_resolution, 
            cell_max_min, skip, thresh, gamma] = load_parameters('sim', p)
    print('Done load_parameters')
    print('cell_resolution: (%d,%d)' % (cell_resolution[0], cell_resolution[1]))
    print(cell_max_min)

    #read data
    g = pd.read_csv(fn_train, delimiter=',').values
    X_all = np.float_(g[:, 0:3])
    Y_all = np.float_(g[:, 3][:, np.newaxis]).ravel() #* 2 - 1

    #thresh = 0
    max_t = len(np.unique(X_all[:, 0]))
    print('Total number of scans: %d' % max_t)
    for ith_scan in range(0, max_t, skip): #every other scan


        # extract data points of the ith scan
        ith_scan_indx = X_all[:, 0] == ith_scan
        print('{}th scan:\n  N={}'.format(ith_scan, np.sum(ith_scan_indx)))
        X_new = X_all[ith_scan_indx, 1:]
        y_new = Y_all[ith_scan_indx]

        if ith_scan == 0:
            # get all data for the first scan and initialize the model
            X, y = X_new, y_new
            bhm_mdl = BayesianHilbertMap(gamma=gamma, grid=None, cell_resolution=cell_resolution, cell_max_min=cell_max_min, X=X, calc_loss=False)
            
            
        else:
            # information filtering
            print X_new
            q_new = bhm_mdl.predict_proba(X_new)[:, 1]
            info_val_indx = np.absolute(q_new - y_new) > thresh
            X, y = X_new[info_val_indx, :], y_new[info_val_indx]

            # Print number of new points used
            points = float(X.shape[0]) / X_new.shape[0]
            print('  {:.2f}% points were used.'.format(points*100))
            """
            pl.close()
            pl.figure(figsize=(25, 5))
            pl.subplot(131)
            pl.scatter(X[:, 0], X[:, 1], c=y, s=100, cmap='jet', edgecolors='')
            pl.xlim([cell_max_min[0], cell_max_min[1]]); pl.ylim([cell_max_min[2], cell_max_min[3]])
            pl.subplot(132)
            pl.scatter(X_new[:, 0], X_new[:, 1], c=np.absolute(q_new - (y_new + 1) / 2), cmap='jet', s=25, marker='8', edgecolors='')
            pl.colorbar()
            pl.xlim([cell_max_min[0], cell_max_min[1]]); pl.ylim([cell_max_min[2], cell_max_min[3]])
            pl.subplot(133)
            pl.scatter(X_new[:, 0], X_new[:, 1], c=util.cross_entropy(y_new, q_new), cmap='jet', s=25, marker='8', edgecolors='')
            pl.colorbar()
            pl.scatter(X_new[info_val_indx, 0], X_new[info_val_indx, 1], cmap='', s=40, marker='8', edgecolors='k')
            pl.xlim([cell_max_min[0], cell_max_min[1]]);  pl.ylim([cell_max_min[2], cell_max_min[3]])
            pl.show()
            """

        # fit the model
        tic = comp_timer.time()
        bhm_mdl.fit(X, y)

        toc = comp_timer.time()
        print("  Time elapsed = {:.0f} ms".format(1000*(toc-tic)))
#
#
#        # query the model
        q_resolution = 0.01
        xx, yy = np.meshgrid(np.arange(cell_max_min[0], cell_max_min[1], q_resolution), np.arange(cell_max_min[2], 
            cell_max_min[3], q_resolution))
        q_x = np.hstack((xx.ravel()[:, np.newaxis], yy.ravel()[:, np.newaxis]))
        q_mn = bhm_mdl.predict_proba(q_x)[:,1]
#
#        # model
        pl.figure(figsize=(30,5))
#
#        # subplot parameters are for the whole plot
#        # rows of subplots, columns of subplots, # of subplot
        pl.subplot(1,3,1)
        pl.scatter(X[:, 0], X[:, 1], c=y, cmap='jet', s=50, edgecolors='')
        pl.colorbar()
        pl.xlim([cell_max_min[0], cell_max_min[1]]); pl.ylim([cell_max_min[2], cell_max_min[3]])
        
        pl.subplot(1,3,2)
#        #pl.scatter(q_x[:, 0], q_x[:, 1], c=(q_mn*2-1), cmap='jet', s=25, 
#        #marker='8', vmin=-1, vmax=1, edgecolors='')
        pl.scatter(q_x[:, 0], q_x[:, 1], c=(q_mn), cmap='jet', s=25, marker='8', vmin=0, vmax=1, edgecolors='')
        pl.colorbar()
        pl.xlim([cell_max_min[0], cell_max_min[1]]); pl.ylim([cell_max_min[2], cell_max_min[3]])
        
        pl.subplot(1,3,3)
        #pl.scatter(q_x[:, 0], q_x[:, 1], c=np.round(q_mn*2-1, 0), cmap='jet', 
#        #s=25, marker='8',edgecolors='')
        pl.scatter(q_x[:, 0], q_x[:, 1], c=np.round(q_mn, 0), cmap='jet', s=25, marker='8',edgecolors='')
        pl.colorbar()
        pl.xlim([cell_max_min[0], cell_max_min[1]]); pl.ylim([cell_max_min[2], cell_max_min[3]])
        pl.savefig(os.path.join(ros_pkg_path, data_dir_str, 'output/sim/seq_map_1iter_t' + str(ith_scan) + '.png'), 
                bbox_inches='tight')
        #pl.show()
#        
        pl.close("all")
#

    print('Done training Hilbert map, writing map to file')
    # Write map 
    writeMap(bhm_mdl, p, os.path.join(ros_pkg_path, 'hilbert_map.csv'))



if __name__ == '__main__':
    main_bhm()
