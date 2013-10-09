import numpy as np, numpy.linalg as nlg
import scipy as scp
import pickle
import pylab as pl
from error_characterization import *
import IPython

trained_data = {}
trained_data["sys_robot_tf"] = {}
trained_data["alphas"] = {}
trained_data["robot_joints"] = {}

arm_side = 'R'

test_data = {}
test_data["camera_poses"] = {}
test_data["robot_poses"] = {}
test_data["robot_joints"] = {}
test_data["camera_to_robot_tf"] = {}
test_data["camera_poses"][arm_side] = {}
test_data["robot_poses"][arm_side] = {}
test_data["robot_joints"][arm_side] = {}

#sample every sub_sample for testing data 
sub_sample = 4


def get_robot_pose(timestamp, robot_poses):
    return min(range(len(robot_poses[arm_side])), key=lambda i: abs(robot_poses[arm_side][i][0] - timestamp))

if arm_side == 'R':
    data = pickle.load(open('data.pkl'))
else:
    data = pickle.load(open('newest_left_arm_data.pkl'))

n_joints = 7 # ignore the last 2 joint (coupled gripper joints)
camera_to_robot_tf = data['camera_to_robot_tf']
robot_to_camera_tf = nlg.inv(camera_to_robot_tf)
camera_poses = []
robot_poses = []
robot_joints = np.empty((0,n_joints))

camera_poses_test = []
robot_poses_test = []
robot_joints_test = []

ts_start = min(data['camera_poses'][arm_side][0][0], data['robot_poses'][arm_side][0][0])
camera_ts = []
i=1; 
for ts_pose in data['camera_poses'][arm_side]:
    # rough way to remove some camera outliers
    if len(camera_poses)>0 and nlg.norm(camera_poses[-1][:3,3]-camera_pose[:3,3])>0.05:
        continue
    if i % sub_sample  != 0:
        camera_ts.append(ts_pose[0]-ts_start)
        camera_pose = ts_pose[1]
        camera_poses.append(ts_pose[1])
        robot_pose_ind = get_robot_pose(ts_pose[0], data['robot_poses'])
        robot_poses.append(data['robot_poses'][arm_side][robot_pose_ind][1])
        robot_joints = np.r_[robot_joints, np.array([data['robot_joints'][arm_side][robot_pose_ind][1][0:n_joints]])]
    else:
        camera_pose = ts_pose[1]
        camera_poses_test.append(ts_pose)
        robot_pose_ind = get_robot_pose(ts_pose[0], data['robot_poses'])
        robot_poses_test.append(data['robot_poses'][arm_side][robot_pose_ind])
        robot_joints_test.append(data['robot_joints'][arm_side][robot_pose_ind])
     
    i = i+1
    
#Assemble test data 
test_data['camera_to_robot_tf'] = camera_to_robot_tf
test_data['camera_poses'][arm_side] = camera_poses_test
test_data['robot_poses'][arm_side] = robot_poses_test
test_data['robot_joints'][arm_side] = robot_joints_test
pickle.dump(test_data, open("test_data.pkl", "wb"))

camera_poses = [camera_to_robot_tf.dot(pose) for pose in camera_poses]



#pl.figure(1)
#pl.plot(camera_ts, [pose[:3,3] for pose in camera_poses])
#pl.plot(camera_ts, [pose[:3,3] for pose in robot_poses])

"""
# START cleanup
# remove data points that were collected when the robot was moving
static_indicator = []
thresh = 0.00001
for i in range(len(robot_poses)):
    if i > 1 and i < len(robot_poses)-2:
        ind0 = (np.abs(robot_poses[i][:3,3] - robot_poses[i-1][:3,3]) < thresh).all()
        ind1 = (np.abs(robot_poses[i+1][:3,3] - robot_poses[i][:3,3]) < thresh).all()
        static_indicator.append(ind0 and ind1)
    else:
        static_indicator.append(False)

red_camera_ts = [ts for (ind, ts) in zip(static_indicator, camera_ts) if ind]
red_camera_poses = [pose for (ind, pose) in zip(static_indicator, camera_poses) if ind]
red_robot_poses = [pose for (ind, pose) in zip(static_indicator, robot_poses) if ind]
red_robot_joints = np.array([pose for (ind, pose) in zip(static_indicator, robot_joints) if ind])
#pl.figure(3)
#pl.plot(red_camera_ts, [camera_to_robot_tf.dot(pose)[:3,3] for pose in red_camera_poses])
#pl.plot(red_camera_ts, [pose[:3,3] for pose in red_robot_poses])


# median filter for xyz coordinates of camera poses
filt_camera_poses = []
red_camera_trans = np.array([pose[:3,3] for pose in red_camera_poses])
for i in range(red_camera_trans.shape[0]):
    filt_camera_trans = scp.median(red_camera_trans[max(i-3,0):min(i+4, red_camera_trans.shape[0]), :], axis=0)
    filt_camera_pose = red_camera_poses[i]
    filt_camera_pose[:3,3] = filt_camera_trans
    filt_camera_poses.append(filt_camera_pose)
#pl.figure(4)
#pl.plot(red_camera_ts, [camera_to_robot_tf.dot(pose)[:3,3] for pose in filt_camera_poses])
#pl.plot(red_camera_ts, [pose[:3,3] for pose in red_robot_poses])

camera_ts = red_camera_ts
camera_poses = filt_camera_poses
robot_poses = red_robot_poses
robot_joints = red_robot_joints
# END cleanup
"""

n_data = len(camera_poses) # number of training data points (should be the same for robot_poses and robot_joints)

# pose error between camera and FK
orig_pose_error = calc_pose_error(camera_poses, robot_poses)
print "original pose error for training data"
print "mean", np.mean(orig_pose_error, axis=0)
print "std", np.std(orig_pose_error, axis=0)
print


# Optimization problem to find better transform
opt_full_tf = False # because we care more about the translation correction and because there is a closed-form solution
if opt_full_tf:
    sys_robot_tf = sys_correct_tf(camera_poses, robot_poses, np.eye(4))
else:
    camera_trans = np.array([pose[:3,3] for pose in camera_poses])
    robot_trans = np.array([pose[:3,3] for pose in robot_poses])
    sys_robot_tf = transformationEstimationSVD(robot_trans, camera_trans)

# systematic corrected robot poses for training data
sys_robot_poses = [sys_robot_tf.dot(robot_pose) for robot_pose in robot_poses]
# check error against same training data for sanity check
sys_pose_error = calc_pose_error(camera_poses, sys_robot_poses)
print "systematic corrected pose error for training data"
print "mean", np.mean(sys_pose_error, axis=0)
print "std", np.std(sys_pose_error, axis=0)
print

#pl.figure(2)
#pl.plot(camera_ts, [pose[:3,3] for pose in camera_poses])
#pl.plot(camera_ts, [pose[:3,3] for pose in sys_robot_poses])


# Gaussian Process regression to estimate and apply the error as a function of the joint angles
# These 2 are equivalent:
#
# (1)
# gp_robot_poses_test = gp_correct_poses(camera_poses, sys_robot_poses, robot_joints, sys_robot_poses_test, robot_joints_test)
#
# (2)
# alphas = gp_correct_poses_precompute(camera_poses, sys_robot_poses, robot_joints)
# gp_robot_poses_test = gp_correct_poses_fast(alphas, robot_joints, sys_robot_poses_test, robot_joints_test)

alphas = gp_correct_poses_precompute(camera_poses, sys_robot_poses, robot_joints)

# systematic and GP corrected robot poses for training data
gp_robot_poses = gp_correct_poses_fast(alphas, robot_joints, sys_robot_poses, robot_joints)
# check error against same training data for sanity check
gp_pose_error = calc_pose_error(camera_poses, gp_robot_poses)
print "systematic and GP corrected pose error for training data"
print "mean", np.mean(gp_pose_error, axis=0)
print "std", np.std(gp_pose_error, axis=0)
print

#pl.figure(3)
#pl.plot(camera_ts, [pose[:3,3] for pose in camera_poses])
#pl.plot(camera_ts, [pose[:3,3] for pose in gp_robot_poses])


plot_translation_poses_nice(camera_ts, camera_poses, camera_ts, robot_poses, sys_robot_poses, gp_robot_poses, split=False)

trained_data["sys_robot_tf"][arm_side] = sys_robot_tf
trained_data["alphas"][arm_side] = alphas
trained_data["robot_joints"][arm_side] = robot_joints

pickle.dump(trained_data, open("newest_trained_data.pkl", "wb"))

