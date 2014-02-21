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
#from ipdb import launch_ipdb_on_exception

import tfx
import tf
import tf.transformations as tft

N_JOINTS = 7
N_DOF = 6
NO_SUBSAMPLING = 1
TRAINING_SUBSAMPLE = 100
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
        self.modelFile = modelFile
        self.testFile = testFile
 
        self.rawData = {}
        self.trainingData = {}
        self.testingData = {}
        self.trainingMode = mode
        
        self.dataKeys = [RavenDataKeys.CAM_POSE_KEY, RavenDataKeys.ROBOT_POSE_KEY, RavenDataKeys.CAM_TS_KEY, RavenDataKeys.ROBOT_TS_KEY]
        
        try:
            if self.modelFile:
                # the training data is used to build a model
                self.trainingData = pickle.load(open(self.modelFile))
                try:
                    self.trainingMode = self.trainingData[LEFT]['training_mode']
                except:
                    try:
                        self.trainingMode = self.trainingData[RIGHT]['training_mode']
                    except:
                        print 'Failed to load training mode for data. Most likely the data is old'
                
                #HACK to remove stupid extra L/R keys from old data
                if len(self.trainingData.keys()) > 2:
                    temp = {}
                    temp[LEFT] = {}
                    temp[RIGHT] = {}
                    for k in self.trainingData.keys():
                        for arm in self.trainingData[k].keys():
                            temp[arm] = self.trainingData[k][arm] # TODO: Remove when data is valid
                    self.trainingData = temp
            if self.testFile:
                self.testingData = pickle.load(open(self.testFile))
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
    
    def matrixError(self, estimatedPoses, truePoses):
        n_poses = len(estimatedPoses)
        n_dim = estimatedPoses[0].shape[0] * estimatedPoses[0].shape[1]
        matrixError = np.zeros((n_poses,n_dim))
        for i in range(n_poses):
            estimatedPose = estimatedPoses[i]
            truePose = truePoses[i]
            diff = truePose - estimatedPose
            matrixError[i,:] = np.reshape(diff, (1, n_dim))
            
        meanError = np.mean(np.abs(matrixError), axis=0)
        medianError = np.median(np.abs(matrixError), axis=0)
        rmsError = np.sqrt(np.mean(np.square(matrixError), axis=0))
        return (meanError, medianError, rmsError)
    
    def componentwiseError(self, estimatedPoses, truePoses):
        n_poses = len(estimatedPoses)
        n_dim = estimatedPoses[0].shape[0] * estimatedPoses[0].shape[1]
        n_trans = 3
        n_rot = 3
        translationError = np.zeros((n_poses, n_trans))
        rotationError = np.zeros((n_poses, n_rot))
        
        for i in range(n_poses):
            estimatedPose = estimatedPoses[i]
            truePose = truePoses[i]
            deltaPose = np.linalg.inv(estimatedPose).dot(truePose)
            
            deltaRotation = np.eye(4,4)
            deltaRotation[:3,:3] = deltaPose[:3,:3]
            
            translationError[i,:] = deltaPose[:3,3].T
            rotationError[i,:] = euler_from_matrix(deltaRotation)
            
        combinedError = np.c_[translationError, rotationError]
        meanError = np.mean(np.abs(combinedError), axis=0)
        medianError = np.median(np.abs(combinedError), axis=0)
        rmsError = np.sqrt(np.mean(np.square(combinedError), axis=0))
        return (meanError, medianError, rmsError)
    
    def loadTrainingData(self, directory, armName, trainingSubsample=TRAINING_SUBSAMPLE):
        if armName not in ARMS:
            print "ERROR: Illegal arm side provided. Acceptable values are:"
            for arm in ARMS:
                print "\t" + arm
            return
        
        # enumerate all files in the target directory
        pickleFiles = []
        for dirName, dirNames, fileNames in os.walk(directory):
            for fileName in fileNames:
                fileRoot, fileExtension = os.path.splitext(fileName)
                if fileExtension == '.pkl':
                    pickleFiles.append(os.path.join(directory, fileName))
        
        # initialize empty list
        self.rawData[armName] = {}
        self.trainingData[armName] = {}
        self.testingData[armName] = {}
        for key in self.dataKeys:
            self.rawData[armName][key] = []
            self.trainingData[armName][key] = []
            self.testingData[armName][key] = []
        self.rawData[armName][RavenDataKeys.CAM_GRAD_KEY] = []
        self.trainingData[armName][RavenDataKeys.CAM_GRAD_KEY] = []
        self.testingData[armName][RavenDataKeys.CAM_GRAD_KEY] = []
        self.rawData[armName][RavenDataKeys.ROBOT_GRAD_KEY] = []
        self.trainingData[armName][RavenDataKeys.ROBOT_GRAD_KEY] = []
        self.testingData[armName][RavenDataKeys.ROBOT_GRAD_KEY] = []
        
        # read in each file
        print 'Reading trajectory data...'
        try:
            for fileName in pickleFiles:
                # load raw data from file
                trajectoryData = pickle.load(open(fileName))
                #IPython.embed()
                for key in self.dataKeys:
                    self.rawData[armName][key].extend(trajectoryData[key][armName][1:]) # only include after timestep 1
                    
                    # get gradients
                    if key == RavenDataKeys.CAM_POSE_KEY:
                        for i in range(len(trajectoryData[key][armName]) - 1):
                            curPose = trajectoryData[key][armName][i]
                            nextPose = trajectoryData[key][armName][i+1]
                            self.rawData[armName][RavenDataKeys.CAM_GRAD_KEY].append(np.linalg.inv(nextPose).dot(curPose))
                    if key == RavenDataKeys.ROBOT_POSE_KEY:
                        for i in range(len(trajectoryData[key][armName]) - 1):
                            curPose = trajectoryData[key][armName][i]
                            nextPose = trajectoryData[key][armName][i+1]
                            self.rawData[armName][RavenDataKeys.ROBOT_GRAD_KEY].append(np.linalg.inv(nextPose).dot(curPose))
                    
                
            # break up into training / test sets
            numDataPoints = len(self.rawData[armName][self.dataKeys[0]])
            for i in range(numDataPoints):
                dataKeysWithGradient = self.dataKeys + [RavenDataKeys.ROBOT_GRAD_KEY, RavenDataKeys.CAM_GRAD_KEY]
                for key in dataKeysWithGradient:
                    if i % trainingSubsample == 0:
                        self.trainingData[armName][key].append(self.rawData[armName][key][i])
                    else:
                        self.testingData[armName][key].append(self.rawData[armName][key][i])           
        except IOError as e:
            print "ERROR:", e
            print "Failed to load raw data file " + filename
        print 'Done reading trajectory data...'
    
    def train(self, plot=True, loghyper=None, crossValidate=False, trainHyper=True, hyperSeed=None):
        """
        Compute a systematic and residual error correction model for each of the training data sets loaded
        """
        trainingErrors = {}
        hyperparams = {}
        for armName in self.trainingData.keys():
            print "Training models for arm " + armName
            
            # 1. original pose error between camera and FK
            trainingCameraPoses = self.trainingData[armName][RavenDataKeys.CAM_POSE_KEY]
            trainingCameraGradients = self.trainingData[armName][RavenDataKeys.CAM_GRAD_KEY]
            trainingRobotPoses = self.trainingData[armName][RavenDataKeys.ROBOT_POSE_KEY]
            trainingRobotGradients = self.trainingData[armName][RavenDataKeys.ROBOT_GRAD_KEY]
            
            testingCameraPoses = self.testingData[armName][RavenDataKeys.CAM_POSE_KEY]
            testingCameraGradients = self.testingData[armName][RavenDataKeys.CAM_GRAD_KEY]
            testingRobotPoses = self.testingData[armName][RavenDataKeys.ROBOT_POSE_KEY]
            testingRobotGradients = self.testingData[armName][RavenDataKeys.ROBOT_GRAD_KEY]
            
            # set the target and samples based on the training mode
            if self.trainingMode == CAM_TO_FK:
                targetPoses = trainingRobotPoses
                inputPoses = trainingCameraPoses
                inputGradients = trainingCameraGradients
            else:
                targetPoses = trainingCameraPoses
                inputPoses = trainingRobotPoses
                inputGradients = trainingRobotGradients
            
            # original error
            (meanError, medianError, rmsError) = self.componentwiseError(inputPoses, targetPoses)
            print '\n'
            print 'Raw data error:'
            print 'Mean:', meanError
            print 'Median:', medianError
            print 'RMS:', rmsError
            print '\n'
            
            # 2. calculate rigid offset
            numDim = 12
            targetStateMatrix = np.zeros((0,numDim))
            for pose in targetPoses:
                targetStateMatrix = np.r_[targetStateMatrix, np.reshape(pose[:3,:], (numDim,1)).T]
            
            inputStateMatrix = np.zeros((0,numDim))
            for pose in inputPoses:
                inputStateMatrix = np.r_[inputStateMatrix, np.reshape(pose[:3,:], (numDim,1)).T]
            
            #rigidCorrection = estimateRigidCorrectionRANSAC(targetPoses, inputPoses)
            translationOffset = 4
            rigidCorrection = transformationEstimationSVD(inputStateMatrix, targetStateMatrix, translationOffset)
            
            # systematic corrected robot poses for training data
            self.trainingData[armName]['rigid_correction'] = rigidCorrection
            sysPredictedTrainingPoses = [rigidCorrection.dot(inputPose) for inputPose in inputPoses]
            self.trainingData[armName]['sys_predicted_poses'] = sysPredictedTrainingPoses
            
            # original error
            (meanError, medianError, rmsError) = self.componentwiseError(sysPredictedTrainingPoses, targetPoses)
            print 'Sys data error:'
            print 'Mean:', meanError
            print 'Median:', medianError
            print 'RMS:', rmsError
            print '\n'
            #IPython.embed()

            # 3. cross-validate to find the max-likelihood parameters
            """
            if crossValidate:
                print 'Running cross-validation...'
                testHyper = [-1] * 7
                training_percent = 1 - VALIDATION_RATIO
                hyper_seed = self.crossValidate(test_hyperparams, target_poses, sys_predicted_poses, input_state_vector, training_percent)
                print 'Validated hyperparams ', hyper_seed
            """
                
            # 4. calculate residual error correction function
            inputStateMatrix = np.zeros((0,numDim))
            inputGradientMatrix = np.zeros((0,numDim))
            for pose in sysPredictedTrainingPoses:
                inputStateMatrix = np.r_[inputStateMatrix, np.reshape(pose[:3,:], (numDim,1)).T]
            for grad in inputGradients:
                inputGradientMatrix = np.r_[inputGradientMatrix, np.reshape(grad[:3,:], (numDim,1)).T]
            inputStateMatrix = np.c_[inputStateMatrix, inputGradientMatrix]
            
            alphas, hyperparams[armName] = gp_train(inputStateMatrix, targetStateMatrix, train_hyper=True, hyper_seed=hyperSeed)    
            # systematic and GP corrected robot poses for training data
            gpPredictedTrainingPoses = gp_predict_poses(alphas, inputStateMatrix, inputStateMatrix, hyperparams[armName])
            
            # original error
            (meanError, medianError, rmsError) = self.componentwiseError(sysPredictedTrainingPoses, targetPoses)
            print 'GP data error:'
            print 'Mean:', meanError
            print 'Median:', medianError
            print 'RMS:', rmsError
            
            # update the internal models
            self.trainingData[armName]["alphas"] = alphas
            self.trainingData[armName]["loghyper"] = hyperparams[armName]
            self.trainingData[armName]['training_mode'] = self.trainingMode

            # 5. plot the translation poses
            if False:
                camera_ts = self.trainingData[arm_side][RavenDataKeys.CAM_TS_KEY]
                plot_translation_poses_nice(camera_ts, target_poses, camera_ts, input_poses, sys_predicted_poses, gp_predicted_poses, split=False, arm_side=arm_side)

            #trainingErrors[armName] = [orig_pose_error, sys_pose_error, gp_pose_error]
            
        return trainingErrors
    
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
        for arm_side in self.trainingData.keys():
            camera_ts = self.testingData[arm_side][RavenDataKeys.CAM_TS_KEY]
            camera_poses = self.testingData[arm_side][RavenDataKeys.CAM_POSE_KEY]
            robot_ts = self.testingData[arm_side][RavenDataKeys.ROBOT_TS_KEY]
            robot_poses = self.testingData[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            camera_poses_train = self.trainingData[arm_side][RavenDataKeys.CAM_POSE_KEY]
            robot_poses_train = self.trainingData[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            
            # set the target and samples based on the training mode
            if self.trainingMode == CAM_TO_FK:
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
        alphas = self.trainingData[arm_side]['alphas']
        loghyper = self.trainingData[arm_side]['loghyper']

        if train_poses is None:
            if self.trainingMode == CAM_TO_FK:
                train_poses = self.trainingData[arm_side][RavenDataKeys.CAM_POSE_KEY]
            elif self.trainingMode == FK_TO_CAM:
                train_poses = self.trainingData[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            else:
                print 'Invalid training mode!'

        input_state_vector = self.generatePoseVector(input_poses) # convert the poses to state vector for training
        train_state_vector = convert_poses_to_vector(train_poses)
        sys_correction_tf = self.trainingData[arm_side]['sys_correction_tf']
                  
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
        camera_poses_train = self.trainingData[arm_side][RavenDataKeys.CAM_POSE_KEY]
        robot_poses_train = self.trainingData[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            
        # set the target and samples based on the training mode
        if self.trainingMode == CAM_TO_FK:
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
        test_data = self.testingData[arm_side]
        
        camera_poses_test = test_data[RavenDataKeys.CAM_POSE_KEY]
        robot_poses_test = test_data[RavenDataKeys.ROBOT_POSE_KEY]
        camera_ts_test = test_data[RavenDataKeys.CAM_TS_KEY]
        robot_ts_test = test_data[RavenDataKeys.ROBOT_TS_KEY]
        camera_poses_train = self.trainingData[arm_side][RavenDataKeys.CAM_POSE_KEY]
        robot_poses_train = self.trainingData[arm_side][RavenDataKeys.ROBOT_POSE_KEY]
            
        # set the target and samples based on the training mode
        if self.trainingMode == CAM_TO_FK:
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
        pickle.dump(self.trainingData, open(train_filename, "wb"))
        pickle.dump(self.testingData, open(test_filename, "wb"))
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
            r.train(crossValidate=cv)
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
    
    
    
