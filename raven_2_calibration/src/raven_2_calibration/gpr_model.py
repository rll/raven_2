#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('raven_2_calibration')

import rospy

import numpy as np, numpy.linalg as nlg
import scipy as scp

import raven_2_utils
from raven_2_utils.raven_constants import *

from error_characterization import *

import argparse
import collections
import matplotlib
import pickle
import pylab as pl
from transformations import euler_from_matrix, euler_matrix, quaternion_from_matrix

import csv
import os
from operator import itemgetter

import IPython
from ipdb import launch_ipdb_on_exception

import tfx
import tf
import tf.transformations as tft

N_JOINTS = 7
N_DOF = 6
NO_SUBSAMPLING = 1
TRAINING_SUBSAMPLE = 2
TRAINING_RATIO = 0.75
VALIDATION_RATIO = 0.75

LEFT = 'L'
RIGHT = 'R'
ARMS = [LEFT, RIGHT]
CAM_TO_FK = 'C'
FK_TO_CAM = 'F'
TRAINING_MODES = [CAM_TO_FK, FK_TO_CAM] # learn mapping CAM_TO_FK or vice versa

CONVERTED_DATA_FILENAME_TEMPLATE = '%s_arm_data_converted.pkl'
DEF_TRAIN_SAVE = 'error_model.pkl'
DEF_TEST_SAVE = 'test_data.pkl'
CSV_HEADER = ['Arm', 'Homing Procedures', 'FK RMS Test Error (m)', 'FK+Sys RMS Test Error (m)', 'FK+Sys+GP RMS Test Error (m)', \
               'Mean X Error (m)', 'Std X Error (m)', 'Mean Y Error (m)', 'Std Y Error (m)', 'Mean Z Error (m)', 'Std Z Error (m)', 'Mean Yaw Error (rad)', 'Std Yaw Error (rad)', 'Mean Pitch Error (rad)', 'Std Pitch Error (rad)', 'Mean Roll Error (rad)', 'Std Roll Error (rad)']

def get_robot_pose(timestamp, robot_poses):
    return min(range(len(robot_poses)), key=lambda i: abs(robot_poses[i][0] - timestamp))

def get_robot_pose_ind(ts, robot_ts):
    return min(range(len(robot_ts)), key=lambda i: abs(robot_ts[i] - ts))

# returns numpy vector for the pose
def pose_to_vector(pose):
    t = pose[:3,3]
    eul = euler_from_matrix(pose[:3,:3])
    
    # correct euler angles to face the camera
    eul = list(eul)
    for i in range(3):
        if eul[i] > np.pi / 2.0:
            eul[i] = eul[i] - np.pi
        elif eul[i] < np.pi / 2.0:
            eul[i] = eul[i] + np.pi
    
    vec = np.r_[t, eul]
    return vec

# return numpy matrix for the pose vector
def vector_to_pose(vector):
    tf = np.eye(4)
    tf[:3,3] = tf[:3,3] + vector[:3]
    tf[:3,:3] = euler_matrix(*vector[3:])[:3,:3].dot(tf[:3,:3])
    return tf

def convert_poses_to_vector(poses):
    return np.array([pose_to_vector(p) for p in poses])

def convert_vector_to_poses(vector):
    return np.array([vector_to_pose(v) for v in vector])

def comp_array(x,y):
    if(nlg.norm(x) > nlg.norm(y)):
        return True
   
    return False

class RavenErrorModel(object):
    
    def __init__(self, modelFile=None, testFile=None, mode=CAM_TO_FK, numPrevPoses=0):
        self.model_file = modelFile
        self.test_file = testFile
 
        self.camera_to_robot_tf = {}
        self.train_data = {}
        self.test_data = {}
        self.training_mode = mode
        self.n_joints = N_JOINTS
        self.numPrevPoses = numPrevPoses

        try:
            if self.model_file:
                # the training data is used to build a model
                self.train_data = pickle.load(open(self.model_file))
                try:
                    self.training_mode = self.train_data[LEFT]['training_mode']
                except:
                    try:
                        self.training_mode = self.train_data[RIGHT]['training_mode']
                    except:
                        print 'Failed to load training mode for data. Most likely the data is old'
                
                #HACK to remove stupid extra L/R keys from old data
                if len(self.train_data.keys()) > 2:
                    temp = {}
                    temp[LEFT] = {}
                    temp[RIGHT] = {}
                    for k in self.train_data.keys():
                        for arm in self.train_data[k].keys():
                            temp[arm] = self.train_data[k][arm] # TODO: Remove when data is valid
                    self.train_data = temp
            if self.test_file:
                self.test_data = pickle.load(open(self.test_file))
        except IOError as e:
            print "ERROR:", e
            print "Failed to load previous testing and training data file"
    
    def __cleanOldData(self, data, arm_side):
        clean_data = {}
        clean_data[LEFT] = {}
        clean_data[RIGHT] = {}
        try:
            for k in data.keys():
                #HACK
                if k != RavenDataKeys.CAM_ROBOT_TF_KEY and k != RavenDataKeys.CONVERTED_KEY:
                    for arm in data[k].keys():
                        clean_data[arm][k] = data[k][arm]
                else:
                    clean_data[arm_side][k] = data[k]
        except:
            print "Data could not be cleaned. Aborting"
            return None
        return clean_data
    
    def __convertRawData(self, data, camera_to_robot_tf, trainingRatio=TRAINING_RATIO, validationRatio=VALIDATION_RATIO, subsampleRate=1):
        camera_poses_train = []
        robot_poses_train = []
        camera_ts_train = []
        robot_ts_train = []
        robot_joints_train = np.empty((0, self.n_joints))
                
        camera_poses_test = []
        robot_poses_test = []
        camera_ts_test = []
        robot_ts_test = []
        robot_joints_test = np.empty((0, self.n_joints))
            
        ts_start = min(data[RavenDataKeys.CAM_POSE_KEY][0][0], data[RavenDataKeys.ROBOT_POSE_KEY][0][0])
        data_cp = data[RavenDataKeys.CAM_POSE_KEY]
        data_cp.sort(key=itemgetter(0))
        prev_cam_pose = camera_to_robot_tf.dot(data_cp[0][1])
        
        print "Read in %d camera poses" %(len(data_cp))
        
        # remove camera outliers, segment data into test and train
        i=0;
        n_datapoints = len(data_cp)
        n_training = trainingRatio * n_datapoints
        for ts_pose in data_cp:
            i = i + 1
            ts_cam = ts_pose[0] - ts_start
            camera_pose_robot_frame = ts_pose[1]
            robot_pose_ind = get_robot_pose(ts_pose[0], data[RavenDataKeys.ROBOT_POSE_KEY])
            ts_robot = data[RavenDataKeys.ROBOT_POSE_KEY][robot_pose_ind][0] - ts_start
            robot_pose_robot_frame = data[RavenDataKeys.ROBOT_POSE_KEY][robot_pose_ind][1]
          
            
            if i % subsampleRate == 0:
                if i < n_training:
                    camera_ts_train.append(ts_cam)
                    camera_poses_train.append(camera_pose_robot_frame)
                    robot_ts_train.append(ts_robot)
                    robot_poses_train.append(robot_pose_robot_frame)
                    robot_joints_train = np.r_[robot_joints_train, \
                                               np.array([data[RavenDataKeys.ROBOT_JOINT_KEY][robot_pose_ind][1][0:self.n_joints]])]
                else:
                    camera_ts_test.append(ts_cam)
                    camera_poses_test.append(camera_pose_robot_frame)
                    robot_ts_test.append(ts_robot)
                    robot_poses_test.append(robot_pose_robot_frame)
                    robot_joints_test = np.r_[robot_joints_test, \
                                              np.array([data[RavenDataKeys.ROBOT_JOINT_KEY][robot_pose_ind][1][0:self.n_joints]])]
                prev_cam_pose = camera_pose_robot_frame
  
        #Assemble test data
        out_train_data = {}
        out_test_data = {}
        
        out_train_data[RavenDataKeys.CAM_ROBOT_TF_KEY] = camera_to_robot_tf
        out_train_data[RavenDataKeys.CAM_POSE_KEY] = camera_poses_train
        out_train_data[RavenDataKeys.ROBOT_POSE_KEY] = robot_poses_train
        out_train_data[RavenDataKeys.ROBOT_JOINT_KEY] = robot_joints_train
        out_train_data[RavenDataKeys.CAM_TS_KEY] = camera_ts_train
        
        # HACK TO REMOVE SHIITY DATA FOR TESTING ONLY
        camera_poses_test.pop()
        robot_poses_test.pop()
        camera_ts_test.pop()
        robot_ts_test.pop()
        
        out_test_data[RavenDataKeys.CAM_ROBOT_TF_KEY] = camera_to_robot_tf
        out_test_data[RavenDataKeys.CAM_POSE_KEY] = camera_poses_test
        out_test_data[RavenDataKeys.ROBOT_POSE_KEY] = robot_poses_test
        out_test_data[RavenDataKeys.ROBOT_JOINT_KEY] = robot_joints_test
        out_test_data[RavenDataKeys.CAM_TS_KEY] = camera_ts_test
        out_test_data[RavenDataKeys.ROBOT_TS_KEY] = robot_ts_test
        
        return out_test_data, out_train_data
    
    def loadData(self, directory, armSide, subsampleRate=None):
        if armSide not in ARMS:
            print "ERROR: Illegal arm side provided. Acceptable values are:"
            for arm in ARMS:
                print "\t" + arm
            return
        
        pickleFiles = []
        for dirName, dirNames, fileName in os.walk(directory):
            fileRoot, fileExtension = os.path.splitext(fileName)
            if fileExtension == '.pkl':
                pickleFiles.append(fileName)
                
        IPython.embed()
            
    def loadTrainingData(self, filename, arm_side, subsampleRate=None):
        """
        Load a raw data file from data_recorder for building a new GPR model. Parititions the data into training
        and test data given a subsample rate. This should not be used to load 'training data' that has already
        been recorded; loadModel serves this purpose
        """
        if arm_side not in ARMS:
            print "ERROR: Illegal arm side provided. Acceptable values are:"
            for arm in ARMS:
                print "\t" + arm
            return
        
        self.raw_data = {}
        try:
            self.raw_data[arm_side] = pickle.load(open(filename))
            try:
                converted = self.raw_data[arm_side][RavenDataKeys.CONVERTED_KEY]
                if converted:
                    if len(self.train_data.keys()) == 0:
                        self.train_data[arm_side] = self.raw_data[arm_side]['train']
                    self.test_data[arm_side] = self.raw_data[arm_side]['test']
                else:
                    raise Exception('Data has not yet been converted')
                
            except:
                # data has not yet been converted
                cleaned_data = self.__cleanOldData(self.raw_data[arm_side], arm_side)
                if cleaned_data:
                    self.raw_data[arm_side] = cleaned_data[arm_side]
                self.camera_to_robot_tf[arm_side] = self.raw_data[arm_side][RavenDataKeys.CAM_ROBOT_TF_KEY]
                
                if subsampleRate == None:
                    subsampleRate = TRAINING_SUBSAMPLE
                self.test_data[arm_side], self.train_data[arm_side] = \
                    self.__convertRawData(self.raw_data[arm_side], self.camera_to_robot_tf[arm_side], subsampleRate=subsampleRate)
    
                converted_data = {'train':self.train_data[arm_side], 'test':self.test_data[arm_side], RavenDataKeys.CONVERTED_KEY:True}
                pickle.dump(converted_data, open(CONVERTED_DATA_FILENAME_TEMPLATE %(arm_side), "wb"))
    
        except IOError as e:
            print "ERROR:", e
            print "Failed to load raw data file " + filename
    
    def train(self, plot=True, opt_full_tf=True, loghyper=None, cross_validate=True, train_hyper=True, hyper_seed=None):
        """
        Compute a systematic and residual error correction model for each of the training data sets loaded
        """
        errors = {}
        hyperparams = {}
        for arm_side in self.train_data.keys():
            print "Training models for arm " + arm_side
            
            # 1. original pose error between camera and FK
            camera_poses = self.train_data[arm_side][RavenDataKeys.CAM_POSE_KEY]
            robot_poses = self.train_data[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            test_camera_poses = self.test_data[arm_side][RavenDataKeys.CAM_POSE_KEY]
            test_robot_poses = self.test_data[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            
            # set the target and samples based on the training mode
            if self.training_mode == CAM_TO_FK:
                target_poses = robot_poses
                input_poses = camera_poses
            else:
                target_poses = camera_poses
                input_poses = robot_poses
            
            input_state_vector = self.generatePoseVector(input_poses) # convert the poses to state vector for training
            orig_pose_error = calc_pose_error(target_poses, input_poses)
            
            # VERBOSE
            print "original pose error for training data"
            print "mean", np.mean(orig_pose_error, axis=0)
            print "std", np.std(orig_pose_error, axis=0)
            print
            # 2. calculate systematic offset
            if opt_full_tf:
                sys_correction_tf = sys_correct_tf(target_poses, input_poses, np.eye(4))
            else:
                target_trans = np.array([pose[:3,3] for pose in target_poses])
                input_trans = np.array([pose[:3,3] for pose in input_poses])
                sys_correction_tf = estimateSystematicOffsetRANSAC(input_trans, target_trans)
            
            # systematic corrected robot poses for training data
            self.train_data[arm_side]['sys_correction_tf'] = sys_correction_tf
            sys_predicted_poses = [sys_correction_tf.dot(input_pose) for input_pose in input_poses]
            self.train_data[arm_side]['sys_predicted_poses'] = sys_predicted_poses
            
            # VERBOSE
            # check error against same training data for sanity check
            sys_pose_error = calc_pose_error(target_poses, sys_predicted_poses)
            print "systematic corrected pose error for training data"
            print "mean", np.mean(sys_pose_error, axis=0)
            print "std", np.std(sys_pose_error, axis=0)
            print
            
            # 3. cross-validate to find the max-likelihood parameters 
            if cross_validate:
                print 'Running cross-validation...'
                test_hyperparams = [-0.5, -1, -1.5, -2, -3, -5, -7.5, -10]
                training_percent = 1 - VALIDATION_RATIO
                hyper_seed = self.crossValidate(test_hyperparams, target_poses, sys_predicted_poses, input_state_vector, training_percent)
                print 'Validated hyperparams ', hyper_seed
                
            # 4. calculate residual error correction function
            alphas, hyperparams[arm_side] = gp_correct_poses_precompute(target_poses, sys_predicted_poses, input_state_vector, train_hyper, hyper_seed=hyper_seed, subsample=10)    
            
            # systematic and GP corrected robot poses for training data
            gp_predicted_poses = gp_correct_poses_fast(alphas, input_state_vector, sys_predicted_poses, input_state_vector, hyperparams[arm_side])
            
            # VERBOSE
            # check error against same training data for sanity check
            gp_pose_error = calc_pose_error(target_poses, gp_predicted_poses)
            print "systematic and GP corrected pose error for training data"
            print "mean", np.mean(gp_pose_error, axis=0)
            print "std", np.std(gp_pose_error, axis=0)
            print

            # update the internal models
            self.train_data[arm_side]["alphas"] = alphas
            self.train_data[arm_side]["loghyper"] = hyperparams[arm_side]
            self.train_data[arm_side]['training_mode'] = self.training_mode

            # 5. plot the translation poses
            if plot:
                camera_ts = self.train_data[arm_side][RavenDataKeys.CAM_TS_KEY]
                plot_translation_poses_nice(camera_ts, target_poses, camera_ts, input_poses, sys_predicted_poses, gp_predicted_poses, split=False, arm_side=arm_side)

            errors[arm_side] = [orig_pose_error, sys_pose_error, gp_pose_error]
            
        return errors
    
    def crossValidate(self, test_hyperparams, target_poses, sys_predicted_poses, input_state_vector, training_percent):
        train_hyper = True    
        n_dim = N_DOF
        
        # generate indices for training, validation set
        indices = np.linspace(0, len(target_poses)-1, len(target_poses))
        np.random.shuffle(indices)
        indices = [int(i) for i in indices]
        
        training_size = int(training_percent*(len(target_poses)-1))
        training_indices = indices[:training_size]
        validation_indices = indices[training_size:]
        
        # partition the training / validation poses    
        target_poses_np = np.array(target_poses)
        sys_pred_poses_np = np.array(sys_predicted_poses)
        target_training_poses = target_poses_np[training_indices]
        sys_predicted_training_poses = sys_pred_poses_np[training_indices]
        input_state_training_vector = input_state_vector[training_indices]
            
        target_validation_poses = target_poses_np[validation_indices]
        sys_predicted_validation_poses = sys_pred_poses_np[validation_indices]
        input_state_validation_vector = input_state_vector[validation_indices]
        
        # compute the mean and std errors for each task variable
        validation_mean_errors = []
        validation_std_errors = []
        for i in range(len(test_hyperparams)):
            hyper_seed = [test_hyperparams[i]] * n_dim
            alphas_cv, hypers_cv = gp_correct_poses_precompute(target_training_poses, sys_predicted_training_poses, input_state_training_vector, train_hyper, hyper_seed=hyper_seed, subsample=1)   
                
            gp_predicted_validation_poses = gp_correct_poses_fast(alphas_cv, input_state_training_vector, sys_predicted_validation_poses, input_state_validation_vector, hypers_cv)
            validation_error = calc_pose_error(target_validation_poses, gp_predicted_validation_poses)
            validation_mean_errors.append(np.mean(validation_error, axis=0))
            validation_std_errors.append(np.std(validation_error, axis=0))
            print 'Mean error for loghyper = %f:' %(hyper_seed[0]), validation_mean_errors[-1]
            print 'Std error for loghyper = %f:' %(hyper_seed[0]), validation_std_errors[-1]
            print
        
        # loop through all choices of hyperparameters and find the minimum error choice for each task variable 
        validated_hyperparams = []    
        for j in range(n_dim):
            lowest_error = np.Infinity
            lowest_error_hyperparam = 0
            
            # find the hyperparameter for this variable with the lowest error
            for i in range(len(validation_mean_errors)):
                if np.abs(validation_mean_errors[i][j]) < lowest_error:
                    lowest_error = np.abs(validation_mean_errors[i][j])
                    lowest_error_hyperparam = test_hyperparams[i]
            
            validated_hyperparams.append(lowest_error_hyperparam)
        
        return validated_hyperparams
            
    def test(self, plot=True, outfilename=None):
        errors = {}
        for arm_side in self.train_data.keys():
            camera_ts = self.test_data[arm_side][RavenDataKeys.CAM_TS_KEY]
            camera_poses = self.test_data[arm_side][RavenDataKeys.CAM_POSE_KEY]
            robot_ts = self.test_data[arm_side][RavenDataKeys.ROBOT_TS_KEY]
            robot_poses = self.test_data[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            camera_poses_train = self.train_data[arm_side][RavenDataKeys.CAM_POSE_KEY]
            robot_poses_train = self.train_data[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            
            # set the target and samples based on the training mode
            if self.training_mode == CAM_TO_FK:
                target_poses = robot_poses
                train_poses = camera_poses_train
                input_poses = camera_poses
                target_ts = robot_ts
                input_ts = camera_ts
            else:
                target_poses = camera_poses
                train_poses = robot_poses_train
                input_poses = robot_poses
                target_ts = camera_ts
                input_ts = robot_ts
            
            gp_predicted_poses, sys_predicted_poses = self.predict(input_poses, arm_side, train_poses=train_poses)
            
            print "Evaluating test data for arm " + arm_side
            
            if plot:
                # Type 1 fonts for publication
                # http://nerdjusttyped.blogspot.sg/2010/07/type-1-fonts-and-matplotlib-figures.html
                matplotlib.rcParams['ps.useafm'] = True
                matplotlib.rcParams['pdf.use14corefonts'] = True
                matplotlib.rcParams['text.usetex'] = True
                plot_translation_poses_nice(target_ts, target_poses, input_ts, input_poses, sys_predicted_poses, gp_predicted_poses, split=True, arm_side=arm_side)

            errors[arm_side] = self.calcErrors(target_poses, input_poses, sys_predicted_poses, gp_predicted_poses, camera_ts, robot_ts)
            if outfilename is not None:
                self.saveErrors(errors[arm_side], outfilename, arm_side) 
        return errors
    
    def predictSinglePose(self, pose, arm_side):
        # Convert pose to numpy matrix
        p = tft.translation_matrix([pose.position.x,pose.position.y,pose.position.z])
        rot = tft.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        input_pose = np.dot(p,rot)

        gpList, sysList = self.predict([input_pose], arm_side)
        return tfx.pose(gpList[0], frame=pose.frame, stamp=pose.stamp), tfx.pose(sysList[0], frame=pose.frame, stamp=pose.stamp)
    
    def predict(self, input_poses, arm_side, train_poses=None):
        alphas = self.train_data[arm_side]['alphas']
        loghyper = self.train_data[arm_side]['loghyper']

        if train_poses is None:
            if self.training_mode == CAM_TO_FK:
                train_poses = self.train_data[arm_side][RavenDataKeys.CAM_POSE_KEY]
            elif self.training_mode == FK_TO_CAM:
                train_poses = self.train_data[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            else:
                print 'Invalid training mode!'

        input_state_vector = self.generatePoseVector(input_poses) # convert the poses to state vector for training
        train_state_vector = convert_poses_to_vector(train_poses)
        sys_correction_tf = self.train_data[arm_side]['sys_correction_tf']
                  
        # systematic corrected robot poses for test data
        print arm_side, sys_correction_tf
        sys_predicted_poses = [sys_correction_tf.dot(input_pose) for input_pose in input_poses]
            
        # systematic and GP corrected robot poses for training data
        gp_predicted_poses = gp_correct_poses_fast(alphas, train_state_vector, sys_predicted_poses, input_state_vector, loghyper)
        return gp_predicted_poses, sys_predicted_poses
    
        

    def predictFromFile(self, filename, arm_side='L', plot=True, outfilename=None):
        raw_data = pickle.load(open(filename))
        cleaned_data = self.__cleanOldData(raw_data, arm_side)
        if cleaned_data:
            raw_data = cleaned_data[arm_side]
        
        camera_to_robot_tf = raw_data[RavenDataKeys.CAM_ROBOT_TF_KEY]
            
        test_data, train_data = \
            self.__convertRawData(raw_data,camera_to_robot_tf, trainingRatio=0)

        camera_ts = test_data[RavenDataKeys.CAM_TS_KEY]
        camera_poses = test_data[RavenDataKeys.CAM_POSE_KEY]
        robot_ts = test_data[RavenDataKeys.ROBOT_TS_KEY]
        robot_poses = test_data[RavenDataKeys.ROBOT_POSE_KEY]
        camera_poses_train = self.train_data[arm_side][RavenDataKeys.CAM_POSE_KEY]
        robot_poses_train = self.train_data[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            
        # set the target and samples based on the training mode
        if self.training_mode == CAM_TO_FK:
            target_poses = robot_poses
            train_poses = camera_poses_train
            input_poses = camera_poses
            target_ts = robot_ts
            input_ts = camera_ts
        else:
            target_poses = camera_poses
            train_poses = robot_poses_train
            input_poses = robot_poses
            target_ts = camera_ts
            input_ts = robot_ts
      
        gp_predicted_poses, sys_predicted_poses = self.predict(input_poses, arm_side, train_poses=train_poses)
        
        if plot:
            # Type 1 fonts for publication
            # http://nerdjusttyped.blogspot.sg/2010/07/type-1-fonts-and-matplotlib-figures.html
            matplotlib.rcParams['ps.useafm'] = True
            matplotlib.rcParams['pdf.use14corefonts'] = True
            matplotlib.rcParams['text.usetex'] = True
            plot_translation_poses_nice(target_ts, target_poses, input_ts, input_poses, sys_predicted_poses, gp_predicted_poses, split=True, arm_side=arm_side)

        errors = self.calcErrors(target_poses, input_poses, sys_predicted_poses, gp_predicted_poses, camera_ts, robot_ts)

        if outfilename is not None:
            self.saveErrors(errors, outfilename, arm_side)
        return gp_predicted_poses, sys_predicted_poses

    def buildMixtureModel(self, filename, arm_side='L', min_c=2, max_c=10):
        self.loadTrainingData(filename, arm_side)
        test_data = self.test_data[arm_side]
        
        camera_poses_test = test_data[RavenDataKeys.CAM_POSE_KEY]
        robot_poses_test = test_data[RavenDataKeys.ROBOT_POSE_KEY]
        camera_ts_test = test_data[RavenDataKeys.CAM_TS_KEY]
        robot_ts_test = test_data[RavenDataKeys.ROBOT_TS_KEY]
        camera_poses_train = self.train_data[arm_side][RavenDataKeys.CAM_POSE_KEY]
        robot_poses_train = self.train_data[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            
        # set the target and samples based on the training mode
        if self.training_mode == CAM_TO_FK:
            train_poses = camera_poses_train
            target_poses = robot_poses_test
            target_ts = robot_ts_test
            input_poses = camera_poses_test
            input_ts = camera_ts_test
        else:
            train_poses = robot_poses_train
            target_poses = camera_poses_test
            target_ts = camera_ts_test
            input_poses = robot_poses_test
            input_ts = robot_ts_test
        
        gp_predicted_poses, sys_predicted_poses = self.predict(input_poses, arm_side, train_poses=train_poses)
            
        # get corresponding robot indices
        robot_inds = []
        for ts in target_ts:
            robot_inds.append(get_robot_pose_ind(ts, input_ts))
                
        sys_residuals = calc_pose_error(target_poses, [sys_predicted_poses[ind] for ind in robot_inds])
        target_pose_vectors = convert_poses_to_vector(target_poses)
        
        plot_componentwise_residuals(sys_residuals, target_pose_vectors)
        
        gmm = build_mixture_model(sys_residuals, target_pose_vectors, min_components = min_c, max_components = max_c, plot=True)
            
        return gmm
      
    def calcErrors(self, target_poses, input_poses, sys_predicted_poses, gp_predicted_poses, target_ts, input_ts):
        robot_inds = []
        for ts in target_ts:
            robot_inds.append(get_robot_pose_ind(ts, input_ts))
        
        # CALCULATE THE ERROR
        target_trans = np.array([pose[:3,3] for pose in target_poses])
        input_trans = np.array([pose[:3,3] for pose in input_poses])
        sys_input_trans = np.array([pose[:3,3] for pose in sys_predicted_poses])
        gp_input_trans = np.array([pose[:3,3] for pose in gp_predicted_poses])
        
        # VERBOSE
        # pose error between camera and FK
        orig_pose_error = calc_pose_error(target_poses, [input_poses[ind] for ind in robot_inds])
        fk_mean_err = np.mean(orig_pose_error, axis=0)
        fk_std_err = np.std(orig_pose_error, axis=0)
        print "original pose error for test data"
        print "mean", fk_mean_err
        print "std", fk_std_err
        fk_err = (target_trans - input_trans[robot_inds,:]).reshape(-1,1)
        fk_rms_err = np.sqrt((fk_err**2).mean())
        print "xyz rms error", fk_rms_err
        print
        
        # systematic corrected robot poses for test data
        sys_pose_error = calc_pose_error(target_poses, [sys_predicted_poses[ind] for ind in robot_inds])
        sys_mean_err = np.mean(sys_pose_error, axis=0) 
        sys_std_err = np.std(sys_pose_error, axis=0)
        print "systematic corrected pose error for test data"
        print "mean", sys_mean_err
        print "std", sys_std_err
        sys_err = (target_trans - sys_input_trans[robot_inds,:]).reshape(-1,1)
        sys_rms_err = np.sqrt((sys_err**2).mean())
        print "xyz rms error", sys_rms_err
        print
        
        # systematic and GP corrected robot poses for test data
        gp_pose_error = calc_pose_error(target_poses, [gp_predicted_poses[ind] for ind in robot_inds])
        gp_mean_err = np.mean(gp_pose_error, axis=0)
        gp_std_err = np.std(gp_pose_error, axis=0)
        print "systematic and GP corrected pose error for test data"
        print "mean", gp_mean_err
        print "std", gp_std_err
        gp_err = (target_trans - gp_input_trans[robot_inds,:]).reshape(-1,1)
        gp_rms_err = np.sqrt((gp_err**2).mean())
        print "xyz rms error", gp_rms_err
        print

        # put the output into a nice lil dictionary (RMS, MEAN, STD)
        error_dict = {}
        error_dict['n'] = len(target_poses)
        error_dict['fk'] = [fk_rms_err, fk_mean_err, fk_std_err]
        error_dict['sys'] = [sys_rms_err, sys_mean_err, sys_std_err]
        error_dict['gp'] = [gp_rms_err, gp_mean_err, gp_std_err]
        return error_dict
    
    def generatePoseVector(self, poseMatrices):
        poseVector = convert_poses_to_vector(poseMatrices) # convert the poses to state vector for training
        
        # convert input pose vectors to vectors of multiple past poses
        n_poses = poseVector.shape[0]
        n_dim = poseVector.shape[1] * (self.numPrevPoses+1)
        stateVector = np.zeros((n_poses, n_dim))
            
        # put past states together into a vector
        for i in range(n_poses):
            if i >= self.numPrevPoses:
                startIndex = i - self.numPrevPoses
                endIndex = i + 1
                pastStates = poseVector[startIndex:endIndex, :]
                pastStatesLinear = np.reshape(pastStates, (1, n_dim))
                stateVector[i,:] = pastStatesLinear
        
        return stateVector    
    
    def save(self, train_filename=DEF_TRAIN_SAVE, test_filename=DEF_TEST_SAVE):
        pickle.dump(self.train_data, open(train_filename, "wb"))
        pickle.dump(self.test_data, open(test_filename, "wb"))
        print "saved"

    def saveErrors(self, error_dict, filename, arm_side):
        try:
            test = open(filename, 'r') # check to see if the file is there!
            with open(filename, 'a') as csvfile:
                data_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                data_writer.writerow([arm_side, 0, error_dict['fk'][0], error_dict['sys'][0], error_dict['gp'][0], \
                                      error_dict['gp'][1][0], error_dict['gp'][2][0],
                                      error_dict['gp'][1][1], error_dict['gp'][2][1],
                                      error_dict['gp'][1][2], error_dict['gp'][2][2],
                                      error_dict['gp'][1][3], error_dict['gp'][2][3],
                                      error_dict['gp'][1][4], error_dict['gp'][2][4],
                                      error_dict['gp'][1][5], error_dict['gp'][2][5] ])

        except IOError as e:
            # File not found, create it
            with open(filename, 'w') as csvfile:
                data_writer = csv.writer(csvfile)
                data_writer.writerow(CSV_HEADER)
                data_writer.writerow([arm_side, 0, error_dict['fk'][0], error_dict['sys'][0], error_dict['gp'][0], \
                                      error_dict['gp'][1][0], error_dict['gp'][2][0],
                                      error_dict['gp'][1][1], error_dict['gp'][2][1],
                                      error_dict['gp'][1][2], error_dict['gp'][2][2],
                                      error_dict['gp'][1][3], error_dict['gp'][2][3],
                                      error_dict['gp'][1][4], error_dict['gp'][2][4],
                                      error_dict['gp'][1][5], error_dict['gp'][2][5] ])
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--mode',nargs='?',default=None)
    parser.add_argument('-l', '--left-arm-data',nargs='?',default=None)
    parser.add_argument('-r', '--right-arm-data',nargs='?',default=None)
    parser.add_argument('-d', '--model-data',nargs='?',default=None)
    parser.add_argument('-t', '--test-data',nargs='?',default=None)   # original test data used to build the model, used for the mixture models
    parser.add_argument('-s', '--spreadsheet',nargs='?',default='backup.csv')
    parser.add_argument('-c', '--cross-validate',nargs='?',default=False)
    
    args = parser.parse_args(rospy.myargv()[1:])
    mode = args.mode
    del args.mode
    left_arm_data = args.left_arm_data
    del args.left_arm_data
    right_arm_data = args.right_arm_data
    del args.right_arm_data
    model_data = args.model_data
    del args.model_data
    test_data = args.test_data
    del args.test_data
    spreadsheet = args.spreadsheet
    del args.spreadsheet
    cv = args.cross_validate
    del args.cross_validate
    
#    with launch_ipdb_on_exception():
    if True:
        if mode == 'train':
            r = RavenErrorModel()
            if left_arm_data:
                r.loadTrainingData(left_arm_data, LEFT, TRAINING_SUBSAMPLE)
            if right_arm_data:
                r.loadTrainingData(right_arm_data, RIGHT, TRAINING_SUBSAMPLE)
            r.train(cross_validate=cv)
            r.test(outfilename=spreadsheet)
            r.save()
        elif mode == 'test':
            r = RavenErrorModel(model_data)
            if left_arm_data:
                print "Testing arm ", LEFT
                r.predictFromFile(left_arm_data, LEFT, outfilename=spreadsheet)
            if right_arm_data:
                print "Testing arm ", RIGHT
                r.predictFromFile(right_arm_data, RIGHT, outfilename=spreadsheet)
        elif mode == 'mixture':
            r = RavenErrorModel(model_data, test_data)
            if left_arm_data:
                print "Building mixture for arm ", LEFT
                r.buildMixtureModel(left_arm_data, arm_side=LEFT, min_c=1, max_c=20)
            if right_arm_data:
                print "Building mixture for arm ", RIGHT
                r.buildMixtureModel(right_arm_data, arm_side=RIGHT, min_c=1, max_c=20)
        else:
            print "Please specify a valid mode: train or test"
    
    
    
