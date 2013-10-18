#!/usr/bin/env python

import numpy as np, numpy.linalg as nlg
import scipy as scp
import argparse
import pickle
import pylab as pl
import rospy
import matplotlib
import collections
from error_characterization import *
import IPython

N_JOINTS = 7
NO_SUBSAMPLING = 1
TRAINING_SUBSAMPLE = 4
LEFT = 'L'
RIGHT = 'R'
ARMS = [LEFT, RIGHT]
DEF_TRAIN_SAVE = 'error_model.pkl'
DEF_TEST_SAVE = 'test_data.pkl'

def get_robot_pose(timestamp, robot_poses):
    return min(range(len(robot_poses)), key=lambda i: abs(robot_poses[i][0] - timestamp))

def get_robot_pose_ind(ts, robot_ts):
    return min(range(len(robot_ts)), key=lambda i: abs(robot_ts[i] - ts))

class RavenErrorModel(object):
    
    def __init__(self, modelFile=None, testFile=None):
        self.model_file = modelFile
        self.test_file = testFile
        
        self.camera_to_robot_tf = {}
        self.train_data = {}
        self.test_data = {}
        self.n_joints = N_JOINTS
        
        try:
            if self.model_file:
                # the training data is used to build a model
                self.train_data = pickle.load(open(self.model_file))
                
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
                if k != 'camera_to_robot_tf':
                    for arm in data[k].keys():
                        clean_data[arm][k] = data[k][arm]
                else:
                    clean_data[arm_side][k] = data[k]
        except:
            print "Data could not be cleaned. Aborting"
            return None
        return clean_data
    
    def __convertRawData(self, data, camera_to_robot_tf, subsampleRate=NO_SUBSAMPLING):
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
        
        ts_start = min(data['camera_poses'][0][0], data['robot_poses'][0][0])
                
        # remove camera outliers, segment data into test and train
        i=1; 
        for ts_pose in data['camera_poses']:
            # rough way to remove some camera outliers
            if len(camera_poses_train)>0 and nlg.norm(camera_poses_train[-1][:3,3]-camera_pose_robot_frame[:3,3])>0.05:
                continue
            ts_cam = ts_pose[0] - ts_start
            camera_pose_robot_frame = camera_to_robot_tf.dot(ts_pose[1])
            robot_pose_ind = get_robot_pose(ts_pose[0], data['robot_poses'])
            ts_robot = data['robot_poses'][robot_pose_ind][0] - ts_start
            robot_pose_robot_frame = data['robot_poses'][robot_pose_ind][1]
            
            if i % subsampleRate != 0:
                camera_ts_train.append(ts_cam)
                camera_poses_train.append(camera_pose_robot_frame)
                robot_ts_train.append(ts_robot)
                robot_poses_train.append(robot_pose_robot_frame)
                robot_joints_train = np.r_[robot_joints_train, \
                                           np.array([data['robot_joints'][robot_pose_ind][1][0:self.n_joints]])]
            else:
                camera_ts_test.append(ts_cam)
                camera_poses_test.append(camera_pose_robot_frame)
                robot_ts_test.append(ts_robot)
                robot_poses_test.append(robot_pose_robot_frame)
                robot_joints_test = np.r_[robot_joints_test, \
                                          np.array([data['robot_joints'][robot_pose_ind][1][0:self.n_joints]])]
            i = i+1
                  
        #Assemble test data
        out_train_data = {}
        out_test_data = {}
        
        out_train_data['camera_to_robot_tf'] = camera_to_robot_tf
        out_train_data['camera_poses'] = camera_poses_train
        out_train_data['robot_poses'] = robot_poses_train
        out_train_data['robot_joints'] = robot_joints_train
        out_train_data['camera_ts'] = camera_ts_train
        
        out_test_data['camera_to_robot_tf'] = camera_to_robot_tf
        out_test_data['camera_poses'] = camera_poses_test
        out_test_data['robot_poses'] = robot_poses_test
        out_test_data['robot_joints'] = robot_joints_test
        out_test_data['camera_ts'] = camera_ts_test
        out_test_data['robot_ts'] = robot_ts_test
        
        return out_test_data, out_train_data
    
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
            cleaned_data = self.__cleanOldData(self.raw_data[arm_side], arm_side)
            if cleaned_data:
                self.raw_data[arm_side] = cleaned_data[arm_side]
            self.camera_to_robot_tf[arm_side] = self.raw_data[arm_side]['camera_to_robot_tf']
            
            if subsampleRate == None:
                subsampleRate = TRAINING_SUBSAMPLE
            self.test_data[arm_side], self.train_data[arm_side] = \
                self.__convertRawData(self.raw_data[arm_side], self.camera_to_robot_tf[arm_side], subsampleRate)
        except IOError as e:
            print "ERROR:", e
            print "Failed to load raw data file " + filename
    
    def train(self, plot=True, opt_full_tf=False, loghyper=None):
        """
        Compute a systematic and residual error correction model for each of the training data sets loaded
        """
        errors = {}
        for arm_side in self.train_data.keys():
            print "Training models for arm " + arm_side
            
            # 1. original pose error between camera and FK
            camera_poses = self.train_data[arm_side]['camera_poses']
            robot_poses = self.train_data[arm_side]['robot_poses']
            robot_joints = self.train_data[arm_side]['robot_joints']
            orig_pose_error = calc_pose_error(camera_poses, robot_poses)
            
            # VERBOSE
            print "original pose error for training data"
            print "mean", np.mean(orig_pose_error, axis=0)
            print "std", np.std(orig_pose_error, axis=0)
            print
            
            # 2. calculate systematic offset
            if opt_full_tf:
                sys_robot_tf = sys_correct_tf(camera_poses, robot_poses, np.eye(4))
            else:
                camera_trans = np.array([pose[:3,3] for pose in camera_poses])
                robot_trans = np.array([pose[:3,3] for pose in robot_poses])
                sys_robot_tf = transformationEstimationSVD(robot_trans, camera_trans)
            
            
            # systematic corrected robot poses for training data
            self.train_data[arm_side]['sys_robot_tf'] = sys_robot_tf
            sys_robot_poses = [sys_robot_tf.dot(robot_pose) for robot_pose in robot_poses]
            self.train_data[arm_side]['sys_robot_poses'] = sys_robot_poses
            
            # VERBOSE
            # check error against same training data for sanity check
            sys_pose_error = calc_pose_error(camera_poses, sys_robot_poses)
            print "systematic corrected pose error for training data"
            print "mean", np.mean(sys_pose_error, axis=0)
            print "std", np.std(sys_pose_error, axis=0)
            print
            
            # 3. calculate residual error correction function
            alphas, loghyper = gp_correct_poses_precompute(camera_poses, sys_robot_poses, robot_joints, loghyper)    
            
            # systematic and GP corrected robot poses for training data
            gp_robot_poses = gp_correct_poses_fast(alphas, robot_joints, sys_robot_poses, robot_joints, loghyper)
            
            # VERBOSE
            # check error against same training data for sanity check
            gp_pose_error = calc_pose_error(camera_poses, gp_robot_poses)
            print "systematic and GP corrected pose error for training data"
            print "mean", np.mean(gp_pose_error, axis=0)
            print "std", np.std(gp_pose_error, axis=0)
            print

            # update the internal models
            self.train_data[arm_side]["alphas"] = alphas
            self.train_data[arm_side]["loghyper"] = loghyper


            # plot the translation poses
            if plot:
                camera_ts = self.train_data[arm_side]['camera_ts']
                plot_translation_poses_nice(camera_ts, camera_poses, camera_ts, robot_poses, sys_robot_poses, gp_robot_poses, split=False)

            errors[arm_side] = [orig_pose_error, sys_pose_error, gp_pose_error]
            
        return errors
    
    def test(self, plot=True):
        errors = {}
        for arm_side in self.train_data.keys():
            sys_robot_tf = self.train_data[arm_side]['sys_robot_tf']
            
            camera_ts = self.test_data[arm_side]['camera_ts']
            camera_poses = self.test_data[arm_side]['camera_poses']
            robot_ts = self.test_data[arm_side]['robot_ts']
            robot_poses = self.test_data[arm_side]['robot_poses']
            robot_joints = self.test_data[arm_side]['robot_joints']
            
            gp_robot_poses, sys_robot_poses = self.predict(robot_poses, robot_joints, arm_side)
            
            print "Evaluating test data for arm " + arm_side
            
            if plot:
                # Type 1 fonts for publication
                # http://nerdjusttyped.blogspot.sg/2010/07/type-1-fonts-and-matplotlib-figures.html
                matplotlib.rcParams['ps.useafm'] = True
                matplotlib.rcParams['pdf.use14corefonts'] = True
                matplotlib.rcParams['text.usetex'] = True
                plot_translation_poses_nice(camera_ts, camera_poses, robot_ts, robot_poses, sys_robot_poses, gp_robot_poses, split=True, arm_side=arm_side)

            errors[arm_side] = self.calcErrors(camera_poses, robot_poses, sys_robot_poses, gp_robot_poses, camera_ts, robot_ts)
        return errors
    
    def predict(self, robot_poses, robot_joints, arm_side):
        alphas = self.train_data[arm_side]['alphas']
        loghyper = self.train_data[arm_side]['loghyper']
        robot_joints_train = self.train_data[arm_side]['robot_joints']
        sys_robot_tf = self.train_data[arm_side]['sys_robot_tf']
                  
        # systematic corrected robot poses for test data
        sys_robot_poses = [sys_robot_tf.dot(robot_pose) for robot_pose in robot_poses]
            
        # systematic and GP corrected robot poses for training data
        gp_robot_poses = gp_correct_poses_fast(alphas, robot_joints_train, sys_robot_poses, robot_joints, loghyper)
        return gp_robot_poses, sys_robot_poses
    
        
    def predictFromFile(self, filename, plot=True):
        gp_robot_poses = []
        sys_robot_poses = []
        arm_side = 'L' # TODO: change to dynamic
        
        raw_data = pickle.load(open(filename))
        cleaned_data = self.__cleanOldData(raw_data, arm_side)
        if cleaned_data:
            raw_data = cleaned_data[arm_side]
        
        camera_to_robot_tf = raw_data['camera_to_robot_tf']
            
        test_data, train_data = \
            self.__convertRawData(raw_data,camera_to_robot_tf)

        camera_ts = test_data['camera_ts']
        camera_poses = test_data['camera_poses']
        robot_ts = test_data['robot_ts']
        robot_poses = test_data['robot_poses']
        robot_joints = test_data['robot_joints']
      
        gp_robot_poses, sys_robot_poses = self.predict(robot_poses, robot_joints, arm_side)
            
        if plot:
            # Type 1 fonts for publication
            # http://nerdjusttyped.blogspot.sg/2010/07/type-1-fonts-and-matplotlib-figures.html
            matplotlib.rcParams['ps.useafm'] = True
            matplotlib.rcParams['pdf.use14corefonts'] = True
            matplotlib.rcParams['text.usetex'] = True
            plot_translation_poses_nice(camera_ts, camera_poses, robot_ts, robot_poses, sys_robot_poses, gp_robot_poses, split=True, arm_side=arm_side)

        self.calcErrors(camera_poses, robot_poses, sys_robot_poses, gp_robot_poses, camera_ts, robot_ts)
        return gp_robot_poses, sys_robot_poses
    
    def calcErrors(self, camera_poses, robot_poses, sys_robot_poses, gp_robot_poses, camera_ts, robot_ts):
        robot_inds = []
        for ts in camera_ts:
            robot_inds.append(get_robot_pose_ind(ts, robot_ts))
        
        # CALCULATE THE ERROR
        camera_trans = np.array([pose[:3,3] for pose in camera_poses])
        robot_trans = np.array([pose[:3,3] for pose in robot_poses])
        sys_robot_trans = np.array([pose[:3,3] for pose in sys_robot_poses])
        gp_robot_trans = np.array([pose[:3,3] for pose in gp_robot_poses])
        
        # VERBOSE
        # pose error between camera and FK
        orig_pose_error = calc_pose_error(camera_poses, [robot_poses[ind] for ind in robot_inds])
        print "original pose error for test data"
        print "mean", np.mean(orig_pose_error, axis=0)
        print "std", np.std(orig_pose_error, axis=0)
        fk_err = (camera_trans - robot_trans[robot_inds,:]).reshape(-1,1)
        fk_rms_err = np.sqrt((fk_err**2).mean())
        print "xyz rms error", fk_rms_err
        print
        
        # systematic corrected robot poses for test data
        sys_pose_error = calc_pose_error(camera_poses, [sys_robot_poses[ind] for ind in robot_inds])
        print "systematic corrected pose error for test data"
        print "mean", np.mean(sys_pose_error, axis=0)
        print "std", np.std(sys_pose_error, axis=0)
        sys_err = (camera_trans - sys_robot_trans[robot_inds,:]).reshape(-1,1)
        sys_rms_err = np.sqrt((sys_err**2).mean())
        print "xyz rms error", sys_rms_err
        print
        
        # systematic and GP corrected robot poses for test data
        gp_pose_error = calc_pose_error(camera_poses, [gp_robot_poses[ind] for ind in robot_inds])
        print "systematic and GP corrected pose error for test data"
        print "mean", np.mean(gp_pose_error, axis=0)
        print "std", np.std(gp_pose_error, axis=0)
        gp_err = (camera_trans - gp_robot_trans[robot_inds,:]).reshape(-1,1)
        gp_rms_err = np.sqrt((gp_err**2).mean())
        print "xyz rms error", gp_rms_err
        print
        return [fk_rms_err, sys_rms_err, gp_rms_err]
    
    def save(self, train_filename=DEF_TRAIN_SAVE, test_filename=DEF_TEST_SAVE):
        pickle.dump(self.train_data, open(train_filename, "wb"))
        pickle.dump(self.test_data, open(test_filename, "wb"))
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--mode',nargs='?',default=None)
    parser.add_argument('-l', '--left_arm_data',nargs='?',default=None)
    parser.add_argument('-r', '--right_arm_data',nargs='?',default=None)
    parser.add_argument('-d', '--model_data',nargs='?',default=None)
    
    args = parser.parse_args(rospy.myargv()[1:])
    mode = args.mode
    del args.mode
    left_arm_data = args.left_arm_data
    del args.left_arm_data
    right_arm_data = args.right_arm_data
    del args.right_arm_data
    model_data = args.model_data
    del args.model_data
    
    if mode == 'train':
        r = RavenErrorModel()
        if left_arm_data:
            r.loadTrainingData(left_arm_data, LEFT, TRAINING_SUBSAMPLE)
        if right_arm_data:
            r.loadTrainingData(right_arm_data, RIGHT, TRAINING_SUBSAMPLE)
        r.train(loghyper = np.array([np.log(1), np.log(1), np.log(np.sqrt(0.01))]))
        r.test()
        r.save()
    elif mode == 'test':
        r = RavenErrorModel(model_data)
        if left_arm_data:
            r.predictFromFile(left_arm_data)
        if right_arm_data:
            r.predictFromFile(right_arm_data)
    else:
        print "Please specify a valid mode: train or test"
    
    
    