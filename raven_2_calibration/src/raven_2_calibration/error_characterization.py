# Import required Python code.
import roslib
roslib.load_manifest('raven_2_calibration')

import numpy as np, numpy.linalg as nlg
import scipy as scp, scipy.optimize as sco
from sklearn import mixture

from GPR import gpr
from Tools.general import feval
import openravepy as rave
from transformations import euler_from_matrix, euler_matrix, quaternion_from_matrix, quaternion_matrix

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg
import pylab as pl
import itertools
import tfx

import csv
import ransac

import IPython

class PoseError(object):
    def __init__(self, matError, transError, rotError):
        self.matrixError = matError
        self.translationError = transError
        self.rotationError = rotError

class LsqModel(object):
    def __init__(self, input_columns, output_columns, translation_offset):
        self.input_columns = input_columns
        self.output_columns = output_columns
        self.translation_offset = translation_offset

    def fit(self, data):
        A = np.vstack(data[:,i] for i in self.input_columns).T
        B = np.vstack(data[:,i] for i in self.output_columns).T
        tf = transformationEstimationSVD(A, B, self.translation_offset)
        pose = tfx.pose(tf)
        vec = np.r_[pose.position.list, pose.orientation.list].T
        return vec

    def get_error(self, data, model):
        # convert model to transform
        pose = tfx.pose(model[:3], model[3:])
        tf = pose.matrix
        A = np.vstack(data[:,i] for i in self.input_columns)
        B = np.vstack(data[:,i] for i in self.output_columns)
        
        A_reshaped = np.zeros((3,0))
        B_reshaped = np.zeros((3,0))
        for i in range(A.shape[1]):
            A_reshaped = np.c_[A_reshaped, np.reshape(A[:,i], (3,4))]
            B_reshaped = np.c_[B_reshaped, np.reshape(B[:,i], (3,4))]
        
        A_ext = np.r_[A_reshaped, np.tile([0,0,0,1],(1,A_reshaped.shape[1]/4))]
        
        B_fit = tf.dot(A_ext);
        B_fit = B_fit[:3,:]
        
        err_per_point = np.sum(np.square(B_reshaped-B_fit), axis=0).T 
        err_per_point = err_per_point.A1
        
        err_per_point = np.reshape(err_per_point, (err_per_point.shape[0]/4,4)).T
        err_per_point = np.sum(err_per_point,axis=0)
      #  IPython.embed()
        return err_per_point

def tf_to_vec(tf):
    return tf[:3,:].reshape(-1,1)

def vec_to_tf(vec):
    tf = vec.reshape(3,4)
    tf = np.r_[tf,np.array([[0,0,0,1]])]
    return tf

"""
def tf_to_pose(tf):
    return np.r_[tf[:3,3], rave.axisAngleFromRotationMatrix(tf[:3,:3])]

def pose_to_tf(pose):
    tf = np.eye(4)
    tf[:3,:3] = rave.rotationMatrixFromAxisAngle(np.array(pose[3:]))
    tf[:3,3] = pose[:3]
    return tf
"""

# fast matrix inversion (special case of homogeneous transform)
def tf_inv(tf):
    R = tf[:3,:3]
    t = tf[:3,3]
    inv = np.eye(4)
    inv[:3,:3] = R.T
    inv[:3,3] = -R.T.dot(t)
    return inv

# plots the xyz coordinates of camera_poses, robot_poses, sys_robot_poses and gp_poses in 3 subplots
# camera_ts are the timestamps of camera poses
# robot_ts are the timestamps of the other poses (robot poses)
def plot_translation_poses_nice(camera_ts, camera_poses, robot_ts, robot_poses, sys_robot_poses, gp_robot_poses, split=False, arm_side='?'):
    f, axarr = pl.subplots(3, sharex=True)
    f.set_size_inches(10, 8, forward=True)
    
    # find the camera indices that defines segments of camera data whose timestamps are not farther apart than split_time
    if split:
        split_time = 1.0;
        split_inds = []
        for i in range(len(camera_ts)):
            if i > 0 and camera_ts[i]-camera_ts[i-1] > split_time:
                split_inds.append(i)
            elif i == len(camera_ts)-1:
                split_inds.append(len(camera_ts))

    camera_trans = np.array([pose[:3,3] for pose in camera_poses])
    robot_trans = np.array([pose[:3,3] for pose in robot_poses])
    sys_robot_trans = np.array([pose[:3,3] for pose in sys_robot_poses])
    gp_robot_trans = np.array([pose[:3,3] for pose in gp_robot_poses])

    #IPython.embed()

    axarr[0].set_title('Errors in the end effector translation estimate for %s arm' %(arm_side), size='large')
    for i in range(3):
        if split:
            start_split_ind = 0
            for split_ind in split_inds:
                if start_split_ind == 0:
                    axarr[i].plot(camera_ts[start_split_ind:split_ind], camera_trans[start_split_ind:split_ind,i], 'b', label='ground truth')
                else:
                    axarr[i].plot(camera_ts[start_split_ind:split_ind], camera_trans[start_split_ind:split_ind,i], 'b')
                start_split_ind = split_ind
        else:
            axarr[i].plot(camera_ts, camera_trans[:,i], 'b', label='ground truth')
        axarr[i].plot(robot_ts, robot_trans[:,i], 'g', label='raw data')
        axarr[i].plot(robot_ts, sys_robot_trans[:,i], 'r', label='systematic correction')
        axarr[i].plot(robot_ts, gp_robot_trans[:,i], 'c', label='systematic and GP correction')
        i2coord = {0:'x', 1:'y', 2:'z'}
        axarr[i].set_ylabel(i2coord[i], fontsize=12)
    #pl.xlim(10, 160)
    pl.xlabel('time (s)', fontsize=12)
    axarr[2].legend(prop={'size':12}, loc='lower left', bbox_to_anchor=(0.75, 0.8)).draggable()
    pl.show()

def plot_translation_poses(ts, poses, c = 'bgr', l = '-'):
    t = np.array([pose[:3,3] for pose in poses])
    pl.plot(ts, t[:,0], c[0]+l)
    pl.plot(ts, t[:,1], c[1]+l)
    pl.plot(ts, t[:,2], c[2]+l)
    
def plot_euler_poses(ts, poses, c = 'bgr', l = '-'):
    eul = np.array([euler_from_matrix(pose[:3,:3]) for pose in poses])
    pl.plot(ts, eul[:,0], c[0]+l)
    pl.plot(ts, eul[:,1], c[1]+l)
    pl.plot(ts, eul[:,2], c[2]+l)

def compare_translation_poses(ts, poses0, poses1, l = '--'):
    t0 = np.array([pose[:3,3] for pose in poses0])
    t1 = np.array([pose[:3,3] for pose in poses1])
    pl.plot(ts, t0[:,0], 'b', ts, t1[:,0], 'b'+l)
    pl.plot(ts, t0[:,1], 'g', ts, t1[:,1], 'g'+l)
    pl.plot(ts, t0[:,2], 'r', ts, t1[:,2], 'r'+l)

def compare_euler_poses(ts, poses0, poses1, l = '--'):
    eul0 = np.array([euler_from_matrix(pose[:3,:3]) for pose in poses0])
    eul1 = np.array([euler_from_matrix(pose[:3,:3]) for pose in poses1])
    pl.plot(ts, eul0[:,0], 'b', ts, eul1[:,0], 'b'+l)
    pl.plot(ts, eul0[:,1], 'g', ts, eul1[:,1], 'g'+l)
    pl.plot(ts, eul0[:,2], 'r', ts, eul1[:,2], 'r'+l)

# pose error takes from tf1 to tf0
# typically, tf0 is ground truth and tf1 is the pose being fixed
def calculatePoseError(tf0, tf1):
    numDimMat = 16
    numDimEul = 6
    numDimQuat = 7
    
    numData = len(tf0) # should be the same for tf1
    matrixPoseError = np.empty((numData, numDimMat))
    translationPoseError = np.empty((numData, numDimMat))
    rotationPoseError = np.empty((numData, numDimMat))
    
    
    for i_data in range(numData):
        subtractedTau = tf0 - tf1
        deltaTau = np.lin_alg.inverse(tf0[i_data]).dot(tf1[i_data])
        diffTranslation = deltaTau[:3,3]
        diffRotation = np.eye(4,4)
        diffRotation[:3,:3] = deltaTau[:3,:3]
        
        diffQuat = quaternion_from_matrix(diffRotation)
        diffEuler = euler_from_matrix(diffRotation)
        
        # flip quaternions on the wrong side of the hypersphere
        if diffQuat[3] < 0:
            diffQuat = -diffQuat
            
        pose_error[i_data,:] = np.r_[diff_translation, diff_rot_rep]
    return pose_error

def apply_pose_error(tf1s, errors, euler_representation=True):
    tf0s = []
    for (tf1, error) in zip(tf1s, errors):
        tf0 = np.eye(4)
        tf0[:3,3] = tf1[:3,3] + error[:3]
        #tf0[:3,:3] = rave.rotationMatrixFromAxisAngle(error[3:]).dot(tf1[:3,:3])
        if euler_representation:
            tf0[:3,:3] = euler_matrix(*error[3:])[:3,:3].dot(tf1[:3,:3])
        else:
            tf0[:3,:3] = quaternion_matrix(*error[3:])[:3,:3].dot(tf1[:3,:3])
        tf0s.append(tf0)
    return tf0s

def estimateRigidCorrectionRANSAC(A, B, pointsPerModel=300, iterations=10, outlierThreshold=0.6, minPointsPercent=0.5):
    n_dim = 12
    input_columns = np.linspace(0,11,n_dim)
    output_columns = np.linspace(12,23,n_dim)
    translation_offset = 4
    model = LsqModel(input_columns, output_columns, translation_offset);

    A_np = np.zeros((n_dim,0))
    for pose in A:
        A_np = np.c_[A_np, np.reshape(pose[:3,:], (12,1))]
    
    B_np = np.zeros((n_dim,0))
    for pose in B:
        B_np = np.c_[B_np, np.reshape(pose[:3,:], (12,1))]

    A_np = A_np.T
    B_np = B_np.T
    data = np.c_[A_np, B_np]
    
    min_points_accept = data.shape[0] * minPointsPercent
    vec = ransac.ransac(data, model, pointsPerModel, iterations, outlierThreshold, minPointsPercent, debug=False)

    pose = tfx.pose(vec[:3], vec[3:])
    tf = np.array(pose.matrix)
    
    return tf


# Finds tf such that B[i,:] = tf.dot(A[i,:]) (least-squares sense)
# http://nghiaho.com/?page_id=671
# Differences between this function and sys_correct_tf
# this function:
#   minimizes error in the pose translation only
#   closed-form solution which is global optimum
# sys_correct_tf:
#   minimizes erros in the pose
#   uses iterative general-purpose solver which might not converge
def transformationEstimationSVD(A, B, translationOffset):
    N = A.shape[0] # should be the same as B.shape[0]
    
    # get translation components
    A_trans = np.zeros((N,0))
    B_trans = np.zeros((N,0))
    A_combined = np.zeros((0,3))
    B_combined = np.zeros((0,3))
    
  #  IPython.embed()
    index = 0
    for i in range(A.shape[1]):
        if (i+1) % translationOffset == 0:
            A_trans = np.c_[A_trans, A[:,i:(i+1)]]
            B_trans = np.c_[B_trans, B[:,i:(i+1)]]
    
    for i in range(A.shape[0]):
        A_pose = np.reshape(A[i,:],(3,4))
        B_pose = np.reshape(B[i,:],(3,4))
        A_combined = np.r_[A_combined, A_pose.T]
        B_combined = np.r_[B_combined, B_pose.T]
        
    centroidA = np.array([np.mean(A_combined, axis=0)])
    centroidB = np.array([np.mean(B_combined, axis=0)])
    centroidAtrans = np.array([np.mean(A_trans, axis=0)])
    centroidBtrans = np.array([np.mean(B_trans, axis=0)])
    
    X = np.zeros((3,4*N))
    Y = np.zeros((3,4*N))
    for i in range(N):
        A_pose = np.reshape(A[i,:], (3,4))
        B_pose = np.reshape(B[i,:], (3,4))
        X[:,4*i:4*(i+1)] = A_pose - np.tile(centroidA.T, (1,4))
        Y[:,4*i:4*(i+1)] = B_pose - np.tile(centroidB.T, (1,4))
        
    H = X.dot(Y.T)
   # IPython.embed()
    U, s, V = nlg.svd(H)
    R = V.T.dot(U.T)
    t = -R.dot(centroidAtrans.T) + centroidBtrans.T

    tf = np.r_[np.c_[R,t], np.array([[0,0,0,1]])]
    return tf

def gp_pred_precompute_alpha(logtheta, covfunc, X, y):
	# compute training set covariance matrix (K)
	K = feval(covfunc, logtheta, X)                     # training covariances
	L = nlg.cholesky(K)                      # cholesky factorization of cov (lower triangular matrix)
	alpha = gpr.solve_chol(L.transpose(),y)         # compute inv(K)*y
	return alpha

# same as gp_pred except that uses the precomputed alpha (what is returned from gp_pred_precompute_alpha())
def gp_pred_fast(logtheta, covfunc, X, alpha, Xstar):
	# (marginal) test predictions (Kss = self-cov; Kstar = corss-cov)
	[Kss, Kstar] = feval(covfunc, logtheta, X, Xstar)   # test covariances (Kss = self covariances, Kstar = cov between train and test cases)
	return np.dot(Kstar.transpose(),alpha)         # predicted means

# gt_poses_train (list of n_data homogeneous TF matrices)
# poses_train (list of n_data homogeneous TF matrices)
# state_train (np.array of shape (n_data x d))
def gp_train(input_state, target_state, train_hyper=True, hyper_seed=None, subsample=1):
    n_input_vars = input_state.shape[1]
    n_output_vars = target_state.shape[1]
    
    alphas = []
    loghyper = hyper_seed
    
    if train_hyper:
        loghyper = []
    if hyper_seed == None or hyper_seed.shape[1] != n_input_vars:
        hyper_seed = [-1] * n_input_vars
    
    # sample X for hyperparam training
    n_samples = input_state.shape[0]

    ## data from a noisy GP
    X = input_state
    X_subsample = np.empty((0, n_input_vars))
    y_subsample = np.empty((0, n_output_vars))
    for i in range(n_samples):
        if i % subsample == 0:
            X_subsample = np.r_[X_subsample, input_state[i:(i+1),:]]
            y_subsample = np.r_[y_subsample, target_state[i:(i+1),:]]
            
    ## DEFINE parameterized covariance funcrion
    covfunc = ['kernels.covSum', ['kernels.covSEiso','kernels.covNoise']]
            
    # sample y for y training
    for i_output_var in range(n_output_vars):
        ### sample observations from the GP
        y_output_var = y_subsample[:,i_output_var]
        y = target_state[:,i_output_var]
        
        ## SET (hyper)parameters
        if train_hyper:
            ## LEARN the hyperparameters if none are provided
            print 'GP: ...training variable ', i_output_var
            ### INITIALIZE (hyper)parameters by -1
            d = X.shape[1]
                
            h = hyper_seed[i_output_var]
            init = h * np.ones((d,1))
            loghyper_var_i = np.array([[h], [h]])
            loghyper_var_i = np.vstack((init, loghyper_var_i))[:,0]
            
            print 'initial hyperparameters: ', np.exp(loghyper_var_i)
            ### TRAINING of (hyper)parameters
            #  IPython.embed()
            #loghyper_var_i = gpr.gp_train(loghyper_var_i, covfunc, X_subsample, y_output_var)
            print 'trained hyperparameters: ',np.exp(loghyper_var_i)
            loghyper.append(loghyper_var_i)
        else:
            h = hyper_seed[i_output_var]
            init = h * np.ones((1,d)) 
            loghyper.append(init)

        ## PREDICTION precomputation
        alphas.append(gp_pred_precompute_alpha(loghyper[i_output_var], covfunc, X, y))
    return alphas, loghyper


# alphas (list of n_task_vars as returned by gp_correct_poses_precompute())
# state_train (np.array of shape (n_data x d))
# poses_test (list of n_test homegeneous TF matrices)
# state_test (np.array of shape (n_test x d))
def gp_predict_poses(alphas, state_train, state_test, loghyper=None, use_same=False):
    n_test_samples = state_test.shape[0]
    n_input_vars = state_test.shape[1]
    n_output_vars = len(alphas)
    
    predicted_mean = np.empty((n_test_samples, n_output_vars))
    
    for i_output_var in range(n_output_vars):
        ## data from a noisy GP
        X = state_train

        ## DEFINE parameterized covariance funcrion
        covfunc = ['kernels.covSum', ['kernels.covSEiso','kernels.covNoise']]
        
        ## SET (hyper)parameters if none provided
        loghyper_var_i = np.array([np.log(1), np.log(1), np.log(np.sqrt(0.01))])
        if loghyper != None:
            if use_same:
                loghyper_var_i = loghyper
            else:
                loghyper_var_i = loghyper[i_output_var] 
                
        ### precomputed alpha
        alpha = alphas[i_output_var]

        ### TEST POINTS
        Xstar = state_test

        ## PREDICTION 
        print 'GP: ...prediction'
        predicted_var = gp_pred_fast(loghyper_var_i, covfunc, X, alpha, Xstar) # get predictions for unlabeled data ONLY
        predicted_mean[:,i_output_var] = predicted_var

    output_poses = []
    for i in range(predicted_mean.shape[0]):
        pose = np.reshape(predicted_mean[i,:], (3,4))
        R_unorth = pose[:3,:3]
        U, s, V = np.linalg.svd(R_unorth)
        pose[:3,:3] = U.dot(V)
        pose = np.r_[pose, np.array([[0, 0, 0, 1]])]
        output_poses.append(pose)

    return output_poses

# gt_poses_train (list of n_data homogeneous TF matrices)
# poses_train (list of n_data homogeneous TF matrices)
# state_train (np.array of shape (n_data x d))
def gp_correct_poses_precompute(gt_poses_train, poses_train, state_train, train_hyper=True, hyper_seed=None, subsample=1):
    n_task_vars = pose_error.shape[1]
    alphas = []
    loghyper = hyper_seed
    
    if train_hyper:
        loghyper = []
    if hyper_seed == None:
        hyper_seed = [-1] * n_task_vars
    
    # sample X for hyperparam training
    n = state_train.shape[0]
    d = state_train.shape[1]

    ## data from a noisy GP
    X = state_train
    X_subsample = np.empty((0, d))
    y_subsample = np.empty((0, n_task_vars))
    for i in range(n):
        if i % subsample == 0:
            X_subsample = np.r_[X_subsample, state_train[i:(i+1),:]]
            y_subsample = np.r_[y_subsample, pose_error[i:(i+1),:]]
            
    ## DEFINE parameterized covariance funcrion
    covfunc = ['kernels.covSum', ['kernels.covSEiso','kernels.covNoise']]
            
    # sample y for y training
    for i_task_var in range(n_task_vars):
        ### sample observations from the GP
        y_task_var = y_subsample[:,i_task_var]
        y = pose_error[:,i_task_var]
        
        ## SET (hyper)parameters (HACK: don't do the rotation angles right now - it doesn't do anything because the measurements are so bad)
        if train_hyper and i_task_var < 3:
            ## LEARN the hyperparameters if none are provided
            print 'GP: ...training variable ', i_task_var
            ### INITIALIZE (hyper)parameters by -1
            d = X.shape[1]
                
            h = hyper_seed[i_task_var]
            init = h * np.ones((d,1))
            loghyper_var_i = np.array([[h], [h]])
            loghyper_var_i = np.vstack((init, loghyper_var_i))[:,0]
            print 'initial hyperparameters: ', np.exp(loghyper_var_i)
            ### TRAINING of (hyper)parameters
            loghyper_var_i = gpr.gp_train(loghyper_var_i, covfunc, X_subsample, y_task_var)
            print 'trained hyperparameters: ',np.exp(loghyper_var_i)
            loghyper.append(loghyper_var_i)
        else:
            h = hyper_seed[i_task_var]
            init = h * np.ones((1,d)) 
            loghyper.append(init)

        ## PREDICTION precomputation
       #IPython.embed()
        alphas.append(gp_pred_precompute_alpha(loghyper[i_task_var], covfunc, X, y))
    return alphas, loghyper

# alphas (list of n_task_vars as returned by gp_correct_poses_precompute())
# state_train (np.array of shape (n_data x d))
# poses_test (list of n_test homegeneous TF matrices)
# state_test (np.array of shape (n_test x d))
def gp_correct_poses_fast(alphas, state_train, poses_test, state_test, loghyper=None, use_same=False):
    n_test = len(poses_test)
    n_task_vars = 6

    MU = np.empty((n_test, n_task_vars))
    print 'HYPER', loghyper

    for i_task_var in range(n_task_vars):
        ## data from a noisy GP
        X = state_train

        ## DEFINE parameterized covariance funcrion
        covfunc = ['kernels.covSum', ['kernels.covSEiso','kernels.covNoise']]
        
        ## SET (hyper)parameters if none provided
        loghyper_var_i = np.array([np.log(1), np.log(1), np.log(np.sqrt(0.01))])
        if loghyper != None:
            if use_same:
                loghyper_var_i = loghyper
            else:
                loghyper_var_i = loghyper[i_task_var] 
                

        #print 'hyperparameters: ', np.exp(logtheta)

        ### precomputed alpha
        alpha = alphas[i_task_var]

        ### TEST POINTS
        Xstar = state_test

        ## PREDICTION 
        print 'GP: ...prediction'
        res = gp_pred_fast(loghyper_var_i, covfunc, X, alpha, Xstar) # get predictions for unlabeled data ONLY
        MU[:,i_task_var] = res

    est_gt_poses_test = apply_pose_error(poses_test, MU, euler_representation=True)

    return est_gt_poses_test

# gt_poses_train (list of n_data homogeneous TF matrices)
# poses_train (list of n_data homogeneous TF matrices)
# state_train (np.array of shape (n_data x d))
# poses_test (list of n_test homegeneous TF matrices)
# state_test (np.array of shape (n_test x d))
def gp_correct_poses(gt_poses_train, poses_train, state_train, poses_test, state_test, logtheta=None):
    print "using gp poses"
    pose_error = calc_pose_error(gt_poses_train, poses_train)

    n_test = len(poses_test)
    n_task_vars = 6

    MU = np.empty((n_test, n_task_vars))
    S2 = np.empty((n_test, n_task_vars))

    for i_task_var in range(n_task_vars):
        ## data from a noisy GP
        X = state_train

        ## DEFINE parameterized covariance funcrion
        covfunc = ['kernels.covSum', ['kernels.covSEiso','kernels.covNoise']]

        #print 'hyperparameters: ', np.exp(logtheta)

        ### sample observations from the GP
        y = np.array([pose_error[:,i_task_var]]).reshape((-1,1))

        ### TEST POINTS
        Xstar = state_test

        ## LEARN hyperparameters if not provided
        if logtheta == None:
            # ***UNCOMMENT THE FOLLOWING LINES TO DO TRAINING OF HYPERPARAMETERS***
            ### TRAINING GP
            print
            print 'GP: ...training'
            ### INITIALIZE (hyper)parameters by -1
            d = X.shape[1]
            init = -1.0*np.ones((d,1))
            loghyper = np.array([[-1.0], [-1.0]])
            loghyper = np.vstack((init, loghyper))[:,0]
            print 'initial hyperparameters: ', np.exp(loghyper)
            ### TRAINING of (hyper)parameters
            logtheta = gpr.gp_train(loghyper, covfunc, X, y)
            print 'trained hyperparameters: ',np.exp(logtheta)

        ## PREDICTION 
        print 'GP: ...prediction'
        results = gpr.gp_pred(logtheta, covfunc, X, y, Xstar) # get predictions for unlabeled data ONLY
        MU[:,i_task_var] = results[0][:,0]
        S2[:,i_task_var:i_task_var+1] = results[1]

    est_gt_poses_test = apply_pose_error(poses_test, MU)
    return est_gt_poses_test

def plot_componentwise_residuals(residuals, poses, state_space_grid=1, n_components=1, cv_type='diag', hist_n_bins=50):

    # for now, plot along x,y plane    
    n_dim = residuals.shape[1]
    n_bins = state_space_grid**3
    xy_poses = poses[:,0:2]
    
    x_min = np.min(poses[:,0])
    x_max = np.max(poses[:,0])
    y_min = np.min(poses[:,1])
    y_max = np.max(poses[:,1])
    z_min = np.min(poses[:,2])
    z_max = np.max(poses[:,2])
    
    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min
    
    x_bounds = [x_min]
    y_bounds = [y_min]
    z_bounds = [z_min]
    
    for i in xrange(state_space_grid):
        x_bounds.append(x_min + (((i+1)*x_range) / state_space_grid))
        y_bounds.append(y_min + (((i+1)*y_range) / state_space_grid))
        z_bounds.append(z_min + (((i+1)*z_range) / state_space_grid))
    
    residual_min = np.min(residuals,0)
    residual_max = np.max(residuals,0)
    
    hist_data = {}
    means = []
    covs = []
    
    for j in xrange(n_dim):     
        #IPython.embed()
        i = 0
        pl.figure(j)
        means.append([])
        covs.append([])
        #IPython.embed()
        for x in xrange(state_space_grid):
            for y in xrange(state_space_grid):
                for z in xrange(state_space_grid):
                    pl.subplot(state_space_grid, n_bins/state_space_grid, i+1)
                    x_indices = (poses[:,0] >= x_bounds[x]) & (poses[:,0] <= x_bounds[x+1])
                    y_indices = (poses[:,1] >= y_bounds[y]) & (poses[:,1] <= y_bounds[y+1])
                    z_indices = (poses[:,2] >= z_bounds[z]) & (poses[:,2] <= z_bounds[z+1])
                    #IPython.embed()
                    means[j].append([])
                    covs[j].append([])
        
                    bin_residuals = residuals[x_indices & y_indices & z_indices, j]
                    n, b, p = mpl.pyplot.hist(bin_residuals, bins=hist_n_bins, range=(residual_min[j], residual_max[j]), normed=True)
                    
                    if bin_residuals.shape[0] > 0:
                        gmm = mixture.GMM(n_components=n_components, covariance_type=cv_type, min_covar = 1e-7)
                        gmm.fit(bin_residuals)
                        
                        #print 'Dimension %d, bin %d' %(j, i)
                        vals = np.linspace(residual_min[j], residual_max[j], hist_n_bins)
                        
                        for a in xrange(n_components):
                            mean = gmm.means_[a]
                            sigma = np.sqrt(gmm.covars_[a])
                            means[j][i].append(mean[0])
                            covs[j][i].append(sigma[0])
                            
                            #print 'mean %f \t covariance %f' %(mean, sigma)
                            pl.plot(vals, mpl.mlab.normpdf(vals, mean, sigma))
                    
                    title = 'Histogram of residual error for dimension %d in bin %d' %(j, i)
                    pl.title(title)
                    i = i+1
    hist_data['means'] = means
    hist_data['covs'] = covs
    
    import pickle
    pickle.dump(hist_data, open('hist_data.pkl', "wb"))
    
    # save in a nice lil csv file
    for j in xrange(n_components):
        with open('hist_data_comp%d.csv' %(j), 'w') as csvfile:
            data_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            data_writer.writerow(['Bin', 'X Mean Error (m)', 'X Std Error (m)', 'Y Mean Error (m)', 'Y Std Error (m)', 'Z Mean Error (m)', 'Z Std Error (m)', \
                                  'Yaw Mean Error (rad)', 'Yaw Std Error (rad)', 'Pitch Mean Error (rad)', 'Pitch Std Error (rad)', 'Roll Mean Error (rad)', 'Roll Std Error (rad)',])
            
            for i in xrange(n_bins):
                data_writer.writerow([i, means[0][i][j], covs[0][i][j], means[1][i][j], covs[1][i][j], means[2][i][j], covs[2][i][j], means[3][i][j], covs[3][i][j], means[4][i][j], covs[4][i][j], means[5][i][j], covs[5][i][j]])
    print 'Saved CSV!'
    pl.show()
        

def build_mixture_model(residuals, poses, min_components=2, max_components=10, plot=False, plot_ellipses=False):
    if min_components > max_components:
        raise Exception('Illegal range for number of mixture components')
    
    # create data matrix
    r_shape = residuals.shape
    p_shape = poses.shape
    X = residuals
    #X = np.append(X, poses, 1)
    
    n_components_range = range(min_components, max_components)
    lowest_bic = np.infty
    bic = []
    cv_types = ['spherical', 'tied', 'diag', 'full']
    for cv_type in cv_types:
        for n_components in n_components_range:
            gmm = mixture.GMM(n_components=n_components, covariance_type=cv_type)
            gmm.fit(X)
            bic.append(gmm.bic(X))
            if bic[-1] < lowest_bic:
                lowest_bic = bic[-1]
                best_gmm = gmm
            

    fig = pl.figure()
    if plot:
        # Plot the BIC scores
        bic = np.array(bic)
        color_iter = itertools.cycle(['k', 'r', 'g', 'b', 'c', 'm', 'y'])
        clf = best_gmm
        bars = []
        
        spl = fig.add_subplot(2,1,1)
        for i, (cv_type, color) in enumerate(zip(cv_types, color_iter)):
            xpos = np.array(n_components_range) + 0.2*(i - 2)
            bars.append(pl.bar(xpos, bic[i * len(n_components_range):
                                 (i + 1) * len(n_components_range)],
                       width=.2, color=color))
        pl.xticks(n_components_range)
        pl.ylim([bic.min() * 1.01 - .01 * bic.max(), bic.max()])
        pl.title('BIC score per model')
        xpos = np.mod(bic.argmin(), len(n_components_range)) + .65 +\
            .2 * np.floor(bic.argmin() / len(n_components_range))
        pl.text(xpos, bic.min() * 0.97 + .03 * bic.max(), '*', fontsize=14)
        spl.set_xlabel('Number of components')
        spl.legend([b[0] for b in bars], cv_types)
        IPython.embed()
        # Plot the points in 3D space with ellipsoids
        splot = fig.add_subplot(2, 1, 2, projection='3d')
        Y_ = clf.predict(X)
  
        clist = []
        mlist = []
        for (i, color) in zip(xrange(clf.n_components), color_iter):
            clist.append(color)
            mlist.append(i)
            print "Color ", i, color
            print "Mean ", i, clf.means_[i]
            print "Cov ", i, clf.covars_[i]
            
            if not np.any(Y_ == i):
                continue
            X_3d_loc = poses[Y_ == i, :3]
            mean = np.mean(X_3d_loc, 0)
            covar = np.cov(np.transpose(X_3d_loc))
            
            l = "mixture%d" %(i)
            splot.scatter(X_3d_loc[:,0], X_3d_loc[:,1], X_3d_loc[:,2], s=1.5, color=color, label=l)
            if plot_ellipses:
                plot_ellipsoid(splot, mean, covar, color)
        
        handles, labels = splot.get_legend_handles_labels()
        splot.legend(handles, labels)
        
        splot.set_xlabel('X (m)')
        splot.set_ylabel('Y (m)')
        splot.set_zlabel('Z (m)')
        splot.set_xlim(np.min(poses[:,0]), np.max(poses[:,0]))
        splot.set_ylim(np.min(poses[:,1]), np.max(poses[:,1]))
        splot.set_zlim(np.min(poses[:,2]), np.max(poses[:,2]))
        pl.title('Selected GMM: full model')
        # pl.subplots_adjust(hspace=.35, bottom=.02)
        pl.show()
    return gmm

def plot_ellipsoid(ax, mean, covar, color):
    U, s, rotation = linalg.svd(covar)
    radii = np.sqrt(s)
    # now carry on with EOL's answer
    u = np.linspace(0.0, 2.0 * np.pi, 100)
    v = np.linspace(0.0, np.pi, 100)
    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
    for i in range(len(x)):
        for j in range(len(x)):
            [x[i,j],y[i,j],z[i,j]] = np.dot([x[i,j],y[i,j],z[i,j]], rotation) + mean
            
    ax.plot_surface(x, y, z,  rstride=4, cstride=4, color=color, alpha=0.2)

def print_formatted_error_stat(error):
	error_stat = ""
	for (mean, std) in zip(np.mean(error, axis=0).tolist(), np.std(error, axis=0).tolist()):
		error_stat += str(mean) + " +/- " + str(std) + "\t"
	print error_stat
    
def expectation_x_given_y_xhat_numerator(sample_poses, target_poses, loghyper, covariance):
    x_est = sample_poses
    x_meas = sample_poses
    y = target_poses

