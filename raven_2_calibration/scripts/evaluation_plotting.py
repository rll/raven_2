import numpy as np, numpy.linalg as nlg
import scipy as scp
import pickle
import pylab as pl
import matplotlib
from error_characterization import *
import IPython

def plot_evaluation(camera_ts, camera_poses, robot_poses, sys_robot_poses, gp_robot_poses):
    # Type 1 fonts for publication
    # http://nerdjusttyped.blogspot.sg/2010/07/type-1-fonts-and-matplotlib-figures.html
    matplotlib.rcParams['ps.useafm'] = True
    matplotlib.rcParams['pdf.use14corefonts'] = True
    matplotlib.rcParams['text.usetex'] = True

    f, axarr = pl.subplots(3, sharex=True)
    f.set_size_inches(10, 8, forward=True)
    split_inds = []
    for i in range(len(camera_ts)):
        if i > 0 and camera_ts[i]-camera_ts[i-1] > 1:
            split_inds.append(i)
        elif i == len(camera_ts)-1:
            split_inds.append(len(camera_ts))
    camera_trans = np.array([pose[:3,3] for pose in camera_poses])
    robot_trans = np.array([pose[:3,3] for pose in robot_poses])
    sys_robot_trans = np.array([pose[:3,3] for pose in sys_robot_poses])
    gp_robot_trans = np.array([pose[:3,3] for pose in gp_robot_poses])
    axarr[0].set_title('Systematic errors in the end effector translation estimate', size='large')
    for i in range(3):
        start_split_ind = 0
        for split_ind in split_inds:
            if start_split_ind == 0:
                axarr[i].plot(camera_ts[start_split_ind:split_ind], camera_trans[start_split_ind:split_ind,i], 'b', label='camera estimate')
            else:
                axarr[i].plot(camera_ts[start_split_ind:split_ind], camera_trans[start_split_ind:split_ind,i], 'b')
            start_split_ind = split_ind
        axarr[i].plot(robot_ts, robot_trans[:,i], 'g', label='forward kinematics (FK)')
        axarr[i].plot(robot_ts, sys_robot_trans[:,i], 'r', label='FK with systematic correction')
        axarr[i].plot(robot_ts, gp_robot_trans[:,i], 'c', label='FK with systematic and GP correction')
        if i==0:
            var = 'x'
        elif i==1:
            var = 'y'
        else:
            var = 'z'
        axarr[i].set_ylabel(var, fontsize=12)
    pl.xlim(10, 160)
    pl.xlabel('time (s)', fontsize=12)
    axarr[2].legend(prop={'size':12}, loc='lower left', bbox_to_anchor=(0.75, 0.8))

