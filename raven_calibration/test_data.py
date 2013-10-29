#!/usr/bin/env python

import numpy as np, numpy.linalg as nlg
import scipy as scp
import argparse
import rospy
import pickle
import pylab as pl
import matplotlib
from error_characterization import *
import IPython

def get_robot_pose(timestamp, robot_poses):
    return min(range(len(robot_poses[arm_side])), key=lambda i: abs(robot_poses[arm_side][i][0] - timestamp))

def convert_test_data(data):
    camera_to_robot_tf = data['camera_to_robot_tf']
    robot_to_camera_tf = nlg.inv(camera_to_robot_tf)
    camera_poses = []
    robot_poses = []
    robot_joints = np.empty((0,n_joints))
    
    camera_poses_test = []
    robot_poses_test = []
    robot_joints_test = []
    
    ts_start = min(data['camera_poses'][arm_side][0][0], data['robot_poses'][arm_side][0][0])
    
    # remove camera outliers, segment data into test and train
    camera_ts = []
    i=1; 
    for ts_pose in data['camera_poses'][arm_side]:
        # rough way to remove some camera outliers
        if len(camera_poses)>0 and nlg.norm(camera_poses[-1][:3,3]-camera_pose[:3,3])>0.05:
            continue

        camera_pose = ts_pose[1]
        camera_poses_test.append(ts_pose)
        robot_pose_ind = get_robot_pose(ts_pose[0], data['robot_poses'])
        robot_poses_test.append(data['robot_poses'][arm_side][robot_pose_ind])
        robot_joints_test.append(data['robot_joints'][arm_side][robot_pose_ind])
        i = i+1
        
    #Assemble test data 
    test_data = {}
    test_data["camera_poses"] = {}
    test_data["robot_poses"] = {}
    test_data["robot_joints"] = {}
    test_data["camera_to_robot_tf"] = {}
    test_data["camera_poses"][arm_side] = {}
    test_data["robot_poses"][arm_side] = {}
    test_data["robot_joints"][arm_side] = {}
    test_data['camera_to_robot_tf'] = camera_to_robot_tf
    test_data['camera_poses'][arm_side] = camera_poses_test
    test_data['robot_poses'][arm_side] = robot_poses_test
    test_data['robot_joints'][arm_side] = robot_joints_test
    return test_data

# Type 1 fonts for publication
# http://nerdjusttyped.blogspot.sg/2010/07/type-1-fonts-and-matplotlib-figures.html
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

arm_side = 'R'
n_joints = 7

parser = argparse.ArgumentParser()
parser.add_argument('arm',nargs='?')
parser.add_argument('train_file_name',nargs='?',default=None)
parser.add_argument('test_file_name',nargs='?',default=None)
parser.add_argument('convert',nargs='?',default=None)
args = parser.parse_args(rospy.myargv()[1:])
arm_side = args.arm or rospy.get_param('~arm','R')
del args.arm
train_file_name = args.train_file_name or None
del args.train_file_name
test_file_name = args.test_file_name or None
del args.test_file_name

prefix = ''
if arm_side == 'R':
    prefix = 'right_'
else:
    prefix = 'left_'
    
if train_file_name == None:
    train_file_name = prefix + 'train_data.pkl'
if test_file_name == None:
    test_file_name = prefix + 'test_data.pkl'
    
trained_data = pickle.load(open(train_file_name))
run_data = pickle.load(open(test_file_name))

if args.convert:
    run_data = convert_test_data(run_data)

# START load test data
camera_to_robot_tf = run_data['camera_to_robot_tf']
robot_to_camera_tf = nlg.inv(camera_to_robot_tf)

# HACK TO GET RID OF LAST CRAPPY DATA POINT
run_data['camera_poses'][arm_side].pop()
run_data['robot_poses'][arm_side].pop()
run_data['robot_joints'][arm_side].pop()

ts_start = min(run_data['camera_poses'][arm_side][0][0], run_data['robot_poses'][arm_side][0][0])
print len(run_data['camera_poses'][arm_side])
camera_ts = []
camera_poses = []
for ts_pose in run_data['camera_poses'][arm_side]:
    camera_ts.append(ts_pose[0]-ts_start)
    camera_poses.append(camera_to_robot_tf.dot(ts_pose[1]))

robot_ts = []
robot_poses = []
for ts_pose in run_data['robot_poses'][arm_side]:
    robot_ts.append(ts_pose[0]-ts_start)
    robot_poses.append(ts_pose[1])

robot_joints = np.empty((0,n_joints))
for ts_joints in run_data['robot_joints'][arm_side]:
    robot_joints = np.r_[robot_joints, np.array([ts_joints[1][0:n_joints]])]
# END load test data


# START trained parameters
# systematic correction
sys_robot_tf = trained_data['sys_robot_tf'][arm_side]

# GP correction
alphas = trained_data['alphas'][arm_side]
loghyper = trained_data['loghyper'][arm_side]
robot_joints_train = trained_data['robot_joints'][arm_side]
# END trained parameters


# systematic corrected robot poses for test data
sys_robot_poses = [sys_robot_tf.dot(robot_pose) for robot_pose in robot_poses]

# systematic and GP corrected robot poses for training data
gp_robot_poses = gp_correct_poses_fast(alphas, robot_joints_train, sys_robot_poses, robot_joints, loghyper, use_same=True)



print "evaluating test data..."

plot_translation_poses_nice(camera_ts, camera_poses, robot_ts, robot_poses, sys_robot_poses, gp_robot_poses, split=True)


# compute various error metrics

# find robot indices that correspond to the available camera poses
def get_robot_pose_ind(ts, robot_ts):
    return min(range(len(robot_ts)), key=lambda i: abs(robot_ts[i] - ts))
robot_inds = []
for ts in camera_ts:
    robot_inds.append(get_robot_pose_ind(ts, robot_ts))

camera_trans = np.array([pose[:3,3] for pose in camera_poses])
robot_trans = np.array([pose[:3,3] for pose in robot_poses])
sys_robot_trans = np.array([pose[:3,3] for pose in sys_robot_poses])
gp_robot_trans = np.array([pose[:3,3] for pose in gp_robot_poses])

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

