# Import required Python code.
import roslib
roslib.load_manifest('raven_2_calibration')

import numpy as np, numpy.linalg as nlg
import scipy as scp, scipy.optimize as sco
from GPR import gpr
from Tools.general import feval
import openravepy as rave
from transformations import euler_from_matrix, euler_matrix
import pylab as pl

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
            axarr[i].plot(camera_ts, camera_trans[:,i], 'b', label='grouth truth')
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
def calc_pose_error(tf0, tf1):
    n_data = len(tf0) # should be the same for tf1
    pose_error = np.empty((n_data, 6))
    for i_data in range(n_data):
        #tf1_to_tf0 = tf0[i_data].dot(tf_inv(tf1[i_data]))
        #pose_error[i_data,:] = tf_to_pose(tf1_to_tf0)
        
        #t = (tf0[i_data][:3,3] - tf1[i_data][:3,3])
        #aa = rave.axisAngleFromRotationMatrix(tf0[i_data][:3,:3].dot(tf1[i_data][:3,:3].T))
        #pose_error[i_data,:] = np.r_[t, aa]
        
        t = (tf0[i_data][:3,3] - tf1[i_data][:3,3])
        euler = euler_from_matrix(tf0[i_data][:3,:3].dot(tf1[i_data][:3,:3].T))
        pose_error[i_data,:] = np.r_[t, euler]
    return pose_error

def apply_pose_error(tf1s, errors):
    tf0s = []
    for (tf1, error) in zip(tf1s, errors):
        tf0 = np.eye(4)
        tf0[:3,3] = tf1[:3,3] + error[:3]
        #tf0[:3,:3] = rave.rotationMatrixFromAxisAngle(error[3:]).dot(tf1[:3,:3])
        tf0[:3,:3] = euler_matrix(*error[3:])[:3,:3].dot(tf1[:3,:3])
        tf0s.append(tf0)
    return tf0s

# Finds tf such that gt_pose = tf.dot(pose)
def sys_correct_tf(gt_poses, poses, tf_init):
	x_init = tf_to_vec(tf_init)

	# Objective function:
	def f_opt (x):
		  n_poses = len(gt_poses)
		  err_vec = np.zeros((0, 1))
		  for i in range(n_poses):
		      err_tf = vec_to_tf(x).dot(poses[i]).dot(tf_inv(gt_poses[i])) - np.eye(4)
		      #err_tf = vec_to_tf(x).dot(poses[i]) - gt_poses[i]
		      err_vec = np.r_[err_vec, tf_to_vec(err_tf)]
		  ret = nlg.norm(err_vec)
		  return ret

	# Rotation constraint:
	def rot_con (x):
		  R = vec_to_tf(x)[0:3,0:3]
		  err_mat = R.T.dot(R) - np.eye(3)
		  ret = nlg.norm(err_mat)
		  return ret

	(X, fx, _, _, _) = sco.fmin_slsqp(func=f_opt, x0=x_init, eqcons=[rot_con], iter=50, full_output=1)

	#print "Function value at optimum: ", fx

	tf = vec_to_tf(np.array(X))
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
def transformationEstimationSVD(A, B):
    N = A.shape[0] # should be the same as B.shape[0]
    centroidA = np.array([np.mean(A, axis=0)])
    centroidB = np.array([np.mean(B, axis=0)])
    H = np.zeros((3,3))
    for i in range(N):
        H += (A[i,:] - centroidA).T.dot(B[i,:] - centroidB)
    U, s, V = nlg.svd(H)
    R = V.T.dot(U.T)
    t = -R.dot(centroidA.T) + centroidB.T

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
def gp_correct_poses_precompute(gt_poses_train, poses_train, state_train, loghyper=None, subsample=1):
    pose_error = calc_pose_error(gt_poses_train, poses_train)
    n_task_vars = pose_error.shape[1]
    alphas = []
    train_hyper = (loghyper == None)
    if train_hyper:
        loghyper = []
    
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
        
        ## SET (hyper)parameters
        if train_hyper:
            ## LEARN the hyperparameters if none are provided
            print 'GP: ...training variable ', i_task_var
            ### INITIALIZE (hyper)parameters by -1
            d = X.shape[1]
                
            init = -1*np.ones((d,1))
            loghyper_var_i = np.array([[-1], [-1]])
            loghyper_var_i = np.vstack((init, loghyper_var_i))[:,0]
            print 'initial hyperparameters: ', np.exp(loghyper_var_i)
            ### TRAINING of (hyper)parameters
            loghyper_var_i = gpr.gp_train(loghyper_var_i, covfunc, X_subsample, y_task_var)
            print 'trained hyperparameters: ',np.exp(loghyper_var_i)
            loghyper.append(loghyper_var_i)

        ## PREDICTION precomputation
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

    est_gt_poses_test = apply_pose_error(poses_test, MU)

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

def print_formatted_error_stat(error):
	error_stat = ""
	for (mean, std) in zip(np.mean(error, axis=0).tolist(), np.std(error, axis=0).tolist()):
		error_stat += str(mean) + " +/- " + str(std) + "\t"
	print error_stat

