#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('raven_2_calibration')

import rospy

import numpy as np, numpy.linalg as nlg
import scipy as scp, scipy.spatial.distance as sdist

import raven_2_utils
from raven_2_utils.raven_constants import *

from error_characterization import *

import argparse
import collections
import matplotlib
import pickle
import pylab as pl
from transformations import euler_from_matrix, euler_matrix, quaternion_from_matrix
import scipy.io

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
TRAINING_SUBSAMPLE = 50
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

TRAINING_INDEX = 0
TESTING_INPUT_INDEX = 1
TESTING_TARGET_INDEX = 2
HYPER_INDEX = 3
ALPHA_INDEX = 4
RIGID_CORRECTION_INDEX = 5
MODEL_NAME = 'errorModelSave'

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

def dataMatrixToPoseList(M):
    poses = []
    deltas = []
    for i in range(M.shape[0]):
        pose = np.reshape(M[i,:16], (4,4))
        poses.append(pose)
        if M.shape[1] > 16:
            delta = np.reshape(M[i,16:], (4,4))
            deltas.append(delta)
    return poses, deltas
    
def poseListToDataMatrix(poses, deltas):
    M = np.zeros((len(poses), 32))
    index = 0
    for pose, delta in zip(poses, deltas):
        M[index,:16] = np.reshape(pose, (1,16))
        M[index,16:] = np.reshape(delta, (1,16))
        index = index+1
    return M

class Hyperparameters(object):
    def __init__(self, mean, covariance, likelihood):
        self.mean = mean
        self.covariance = covariance
        self.likelihood = likelihood

class PredictionError(object):
    def __init__(self, medianError, meanError, rmsError, maxError):
        self.medianError = medianError
        self.meanError = meanError
        self.rmsError = rmsError
        self.maxError = maxError
        
    def __str__(self):
        output = ''
        output += 'Mean:' + str(self.meanError) + '\n'
        output += 'Median:' + str(self.medianError) + '\n'
        output += 'RMS:' + str(self.rmsError) + '\n'
        output += 'Max:' + str(self.maxError) + '\n'
        return output

IND = 10000

class RavenErrorModel(object):
    
    def __init__(self, leftModelFile=None, rightModelFile=None):
        self.leftModelFile = leftModelFile
        self.rightModelFile = rightModelFile
        
        self.trainingInputs = {}
        self.testingInputs = {}
        self.testingTargets = {}
        self.hyperparameters = {}
        self.alphas = {}
        self.rigidCorrection = {}
           
        try:
            if self.leftModelFile:
                arm = 'L'
                model = scipy.io.loadmat(self.leftModelFile)
                self.trainingInputs[arm]  = model[MODEL_NAME][0][TRAINING_INDEX]
                self.testingInputs[arm]   = model[MODEL_NAME][0][TESTING_INPUT_INDEX][:IND,:]
                self.testingTargets[arm]  = model[MODEL_NAME][0][TESTING_TARGET_INDEX][:IND,:]
                self.hyperparameters[arm] = model[MODEL_NAME][0][HYPER_INDEX]
                self.alphas[arm]          = model[MODEL_NAME][0][ALPHA_INDEX]
                self.rigidCorrection[arm] = model[MODEL_NAME][0][RIGID_CORRECTION_INDEX]
                
                self.hyperparameters[arm] = self.convertHyperparams(self.hyperparameters[arm])
                self.alphas[arm] = self.convertAlphas(self.alphas[arm])
            
            if self.rightModelFile:
                arm = 'R'
                model = scipy.io.loadmat(self.rightModelFile)
                self.trainingInputs[arm]  = model[MODEL_NAME][0][TRAINING_INDEX]
                self.testingInputs[arm]   = model[MODEL_NAME][0][TESTING_INPUT_INDEX][:IND,:]
                self.testingTargets[arm]  = model[MODEL_NAME][0][TESTING_TARGET_INDEX][:IND,:]
                self.hyperparameters[arm] = model[MODEL_NAME][0][HYPER_INDEX]
                self.alphas[arm]          = model[MODEL_NAME][0][ALPHA_INDEX]
                self.rigidCorrection[arm] = model[MODEL_NAME][0][RIGID_CORRECTION_INDEX]
                
                self.hyperparameters[arm] = self.convertHyperparams(self.hyperparameters[arm])
                self.alphas[arm] = self.convertAlphas(self.alphas[arm])
                
                #IPython.embed()
        except IOError as e:
            print "ERROR:", e
            print "Failed to load previous testing and training data file"
    
    def convertHyperparams(self, rawHyperMat):
        cleanedHypers = []
        for i in range(rawHyperMat.shape[1]):
            curHyper = rawHyperMat[0,i]
            mean = curHyper[0][0][0]
            cov = curHyper[0][0][1]
            likelihood = curHyper[0][0][2]
            cleanHyper = Hyperparameters(mean, cov, likelihood)
            cleanedHypers.append(cleanHyper)
        return cleanedHypers
    
    def convertAlphas(self, rawAlphas):
        cleanedAlphas = []
        for i in range(rawAlphas.shape[1]):
            curAlpha = rawAlphas[0,i]
            cleanedAlphas.append(curAlpha)
        return cleanedAlphas
    
    def meanSum(self, hyp, x):
        A = x.dot(hyp[0:(x.shape[1])])
        A = A + hyp[(x.shape[1]):]*np.ones((x.shape[0],1))
        return A
    
    def covarianceMaternIso(self, d, hyp, x=None, z=None, i=None):
        if x is None and z is None and i is None:
            K = '2'
            return K
        diagonal = False
        if z == 'diag':
            diagonal = True
        
        ell = np.exp(hyp[0])
        sf2 = np.exp(2*hyp[1])
        
        if d not in [1, 3, 5]:
            print 'Illegal d'
            return None
        
        f = None
        if d == 1:
            f = lambda t: 1
            df = lambda t: 1
        elif d == 3:
            f = lambda t: 1 + t
            df = lambda t: t
        elif d ==5:
            f = lambda t: 1 + t * (1 + t/3.0)
            df = lambda t: t * (1 + t/3.0)
        
        m = lambda t, f: f(t)*np.exp(-t)
        dm = lambda t, f: df(t)*np.exp(-t)*t
        
        if diagonal:
            K = np.zeros(x.shape[0],1)
        else:
            if z.shape[0] == 0:
                K = sdist.pdist((np.sqrt(d) / ell) * x)
                K = sdist.squareform(K)
            else:
                K = sdist.cdist((np.sqrt(d) / ell) * x, (np.sqrt(d) / ell) * z)
        
        if i is None:
            K = sf2 * m(K, f)
        else:
            if i == 1:
                K = sf2*dm(K, f)
            elif i ==2:
                K = 2*sf2*m(K, f)
            else:
                print 'Unknown hyperparameter'
                return None
        return K
    
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
        maxError = np.max(np.abs(matrixError), axis=0)
        
        predictionError = PredictionError(medianError, meanError, rmsError, maxError)
        return predictionError
    
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
            
            #if np.abs(translationError[i,1]) > 0.01:
            #    print 'Outlier', i
            
        combinedError = np.c_[translationError, rotationError]
        meanError = np.mean(np.abs(combinedError), axis=0)
        medianError = np.median(np.abs(combinedError), axis=0)
        rmsError = np.sqrt(np.mean(np.square(combinedError), axis=0))
        maxError = np.max(np.abs(combinedError), axis=0)
        
        predictionError = PredictionError(medianError, meanError, rmsError, maxError)
        return predictionError
            
    # testing should already have been performed in matlab, this function tests that our prediction method is valid
    def test(self, plot=True, outfilename=None):
        errors = {}
        for armName in self.trainingInputs.keys():
            testingInputs = self.testingInputs[armName]
            testingTargets = self.testingTargets[armName]
            
            testingInputPoses, testingInputDeltas = dataMatrixToPoseList(testingInputs)
            testingTargetPoses, testingTargetDeltas = dataMatrixToPoseList(testingTargets)
           
            originalError= self.componentwiseError(testingInputPoses, testingTargetPoses)
            print '\n'
            print 'Raw data error:'
            print originalError
            print '\n'
           
            print "Evaluating test data for arm " + armName 
            gpPredictedTestingPoses, sysPredictedTestingPoses = self.predict(armName, testingInputPoses, testingInputDeltas)
            
            IPython.embed() 

            # rigid correction error
            sysError = self.componentwiseError(sysPredictedTestingPoses, testingTargetPoses)
            print 'Rigid Correction error:'
            print sysError
            print '\n'
            # gp error
            print 'GAUSSIAN PROCESS'
            gpError = self.componentwiseError(gpPredictedTestingPoses, testingTargetPoses)
            print 'GP error:'
            print gpError
            print '\n'
            
            if plot:
                # Type 1 fonts for publication
                # http://nerdjusttyped.blogspot.sg/2010/07/type-1-fonts-and-matplotlib-figures.html
                matplotlib.rcParams['ps.useafm'] = True
                matplotlib.rcParams['pdf.use14corefonts'] = True
                matplotlib.rcParams['text.usetex'] = True
                fake_camera_ts = [i for i in range(len(testingInputPoses))]
                plot_translation_poses_nice(fake_camera_ts, testingTargetPoses, fake_camera_ts, testingInputPoses, sysPredictedTestingPoses, gpPredictedTestingPoses, split=True, arm_side=armName)

            errors[armName] = [originalError, sysError, gpError]
            #if outfilename is not None:
            #   self.saveErrors(errors[arm_side], outfilename, arm_side) 
        return errors

    def gpPredict(self, armName, testingInputMatrix):
        trainingInputMatrix = self.trainingInputs[armName]
        alphas = self.alphas[armName]
        hyperparams = self.hyperparameters[armName]
        n_dim = 12
        
        testingPredictedMatrix = np.zeros(testingInputMatrix.shape)
        
        for d in range(n_dim):
            #print 'Predicting dimension', d
            Mz = self.meanSum(hyperparams[d].mean, testingInputMatrix)
            Kxz = self.covarianceMaternIso(3, hyperparams[d].covariance, trainingInputMatrix, testingInputMatrix)
            #IPython.embed()

            testingPredictedMatrix[:,d:(d+1)] = Mz + Kxz.T.dot(alphas[d])
            
        # add ones to make them legit poses
        testingPredictedMatrix[:,15:16] = np.ones((testingPredictedMatrix.shape[0],1))
            
        return testingPredictedMatrix
       
    def predict(self, armName, inputPoses, inputGradients):
        rigidCorrection = self.rigidCorrection[armName]
                      
        # systematic corrected robot poses for test data
        #print 'Correcting rigid offset'
        rigidCorrectedTestingPoses = [rigidCorrection.dot(inputPose) for inputPose in inputPoses]
     
        # training state matrix fill-in
        testingInputMatrix = poseListToDataMatrix(rigidCorrectedTestingPoses, inputGradients)
           
        # systematic and GP corrected robot poses for training data
        #print 'Correcting residual errors'
        gpPredictedTestingMatrix = self.gpPredict(armName, testingInputMatrix)
        gpPredictedTestingPoses, dummy = dataMatrixToPoseList(gpPredictedTestingMatrix)
  
        # reorth the gp predictions since they may not be legit
        for i in range(len(gpPredictedTestingPoses)):
            R = gpPredictedTestingPoses[i][:3,:3]
            U, s, V = np.linalg.svd(R)
            R = U.dot(V)
            gpPredictedTestingPoses[i][:3,:3] = R

        return gpPredictedTestingPoses, rigidCorrectedTestingPoses      


    def predictSinglePose(self, armName, curPose, prevPose, dt=1.0):
        if dt <= 0:
            print 'Error: Illegal timestamp'
            return None

        # Convert pose to numpy matrix
        curTrans = tft.translation_matrix([curPose.position.x, curPose.position.y, curPose.position.z])
        curRot = tft.quaternion_matrix([curPose.orientation.x, curPose.orientation.y ,curPose.orientation.z, curPose.orientation.w])
        curPoseMatrix = np.dot(curTrans, curRot)

        prevTrans = tft.translation_matrix([prevPose.position.x, prevPose.position.y, prevPose.position.z])
        prevRot = tft.quaternion_matrix([prevPose.orientation.x, prevPose.orientation.y ,prevPose.orientation.z, prevPose.orientation.w])
        prevPoseMatrix = np.dot(prevTrans, prevRot)
        
        deltaPoseMatrix = np.linalg.inv(prevPoseMatrix).dot(curPoseMatrix)
        deltaAngles = euler_from_matrix(deltaPoseMatrix[:3,:3])
        deltaPos = deltaPoseMatrix[:3,3]

        #deltaAngles = np.array([a / dt for a in deltaAngles])
        deltaPos = deltaPos / dt
        #deltaPoseMatrix = euler_matrix(deltaAngles[0], deltaAngles[1], deltaAngles[2])
        deltaPoseMatrix[:3,3] = deltaPos

        gpList, sysList = self.predict(armName, [curPoseMatrix], [deltaPoseMatrix])
        return tfx.pose(gpList[0], frame=curPose.frame, stamp=curPose.stamp), tfx.pose(sysList[0], frame=curPose.frame, stamp=curPose.stamp)
 
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
    parser.add_argument('arm',nargs='?',default='L')
    parser.add_argument('-m', '--mode',nargs='?',default=None)
    parser.add_argument('-d', '--model-data',nargs='?',default=None)
    parser.add_argument('-s', '--spreadsheet',nargs='?',default='backup.csv')
    args = parser.parse_args(rospy.myargv()[1:])
    
    armName = args.arm
    del args.arm
    mode = args.mode
    del args.mode
    modelData = args.model_data
    del args.model_data
    spreadsheet = args.spreadsheet
    del args.spreadsheet
    
#    with launch_ipdb_on_exception():
    if True:
        if mode == 'test':
            r = None
            if armName == 'L':
                r = RavenErrorModel(leftModelData=modelData)
            else:
                r = RavenErrorModel(rightModelData=modelData)
                    
            r.test()
        else:
            #print "Please specify a valid mode: train or test"
            try:
                for i in range(1):
                    r = None
                    if armName == 'L':
                        r = RavenErrorModel(leftModelData=modelData)
                    else:
                        r = RavenErrorModel(rightModelData=modelData)
                    data = pickle.load(open('/home/jmahler/ros_workspace/RavenDebridement/test_%d.pkl' %(i) ))
                    prevPose = tfx.pose([0,0,0])
                    
                    sysPredPoses = []
                    gpPredPoses = []
                    actualPoses = []
                    cameraPoses = []
                    count = 0
                    tp = data['target_pose'][armName]
                    IPython.embed()
                        
                    for cp, rp in zip(data['camera_poses'][armName], data['command_poses'][armName]):
                        pose = tfx.pose(np.reshape(cp, (4,4)))
                        if count > 0 and np.abs(rp[2,3]) > 0.01:
                            gpPred, sysPred = r.predictSinglePose(armName, pose, prevPose)
                            sysPredPoses.append(tp)
                            gpPredPoses.append(gpPred.matrix)
                            actualPoses.append(rp)
                            cameraPoses.append(cp)
                        prevPose = pose
                        count = count+1

                    pose = tfx.pose(tp)
                    gpPred, sysPred = r.predictSinglePose(armName, pose, pose)
                    test = gpPred.matrix
                    #gpPredPoses = [test for i in range(len(sysPredPoses))]

                    p = r.componentwiseError(gpPredPoses, actualPoses)
                    print p


                    matplotlib.rcParams['ps.useafm'] = True
                    matplotlib.rcParams['pdf.use14corefonts'] = True
                    matplotlib.rcParams['text.usetex'] = True
                    fake_camera_ts = [i for i in range(len(gpPredPoses))]
                    plot_translation_poses_nice(fake_camera_ts, actualPoses, fake_camera_ts, cameraPoses,
                        sysPredPoses, gpPredPoses, split=True, arm_side=armName)


            except:
                print 'Done reading files'
