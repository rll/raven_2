#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import numpy
import os
import pickle
import scipy.io
from random import shuffle

def get_robot_pose_ind(ts, robot_ts):
	return min(range(len(robot_ts)), key=lambda i: abs(robot_ts[i] - ts))

parser = argparse.ArgumentParser()
    
parser.add_argument('arm',nargs='?',default=None)
parser.add_argument('-f','--filename',nargs='?',default=None)
parser.add_argument('-d','--directory',nargs='?',default='trajectory_data_phasespace')

args = parser.parse_args()

arms = [args.arm]
del args.arm
filename = args.filename
del args.filename
directory = args.directory
del args.directory

pickleFiles = []
if filename is not None:
	pickleFiles.append(filename)
else:
	for dirName, dirNames, fileNames in os.walk(directory):
		for fileName in fileNames:
			fileRoot, fileExtension = os.path.splitext(fileName)
			if fileExtension == '.pkl':
				pickleFiles.append(os.path.join(directory, fileName))

if not arms[0]:
	arms = ['L', 'R']

for filename in pickleFiles:
	file = open(filename, 'r')
	data = pickle.load(file)

	for arm in arms:
		data_r = []
		data_c = []
		data_z = []
		data_t = []
		# timestamps
		data_rs = []
		data_cs = []
		data_zs = []
		use_commands = False

		camera_poses = data['camera_poses'][arm]
		camera_ts = data['camera_ts'][arm]
		robot_poses = data['robot_poses'][arm]
		robot_ts = data['robot_ts'][arm]

		if 'command_poses' in data.keys():
			use_commands = True
			command_poses = data['command_poses'][arm]
			command_ts = data['command_ts'][arm]
		if 'target_pose' in data.keys():
			target_pose = data['target_pose'][arm]
			data_t.append(target_pose)

		# synchronize if necessary
		if len(camera_poses) != len(robot_poses) or (len(camera_poses) > 0 and type(camera_poses[0]) is tuple):
            		camera_ts = [pose[0] for pose in camera_poses]
            		camera_poses_unsync = [pose[1] for pose in camera_poses]
            		robot_ts = [pose[0] for pose in robot_poses]
            		robot_poses_unsync = [pose[1] for pose in robot_poses]

           		# match first timestamps
            		if camera_ts[0] > robot_ts[0]:
                		while camera_ts[0] > robot_ts[0]:
                    			robot_ts.pop(0)
              	    			robot_poses_unsync.pop(0)
            		else:
               			while camera_ts[0] < robot_ts[0]:
                    			camera_ts.pop(0)
                    			camera_poses_unsync.pop(0)

            		# get corresponding robot indices
            		robot_inds = []
            		for ts in camera_ts:
                		robot_inds.append(get_robot_pose_ind(ts, robot_ts))

            		robot_ts_sync = []
            		robot_poses_sync = []
          		for ind in robot_inds:
                		robot_ts_sync.append(robot_ts[ind])
                		robot_poses_sync.append(robot_poses_unsync[ind])

			camera_poses = camera_poses_unsync
			robot_poses = robot_poses_sync
 
 
 		if use_commands:
 			data_list = zip(camera_poses, camera_ts, robot_poses, robot_ts, command_poses, command_ts)
			for cPose, cTs, rPose, rTs, zPose, zTs in data_list:
				rPose = numpy.reshape(rPose,(16,1))
				cPose = numpy.reshape(cPose,(16,1))
				zPose = numpy.reshape(zPose,(16,1))
				data_r.append(rPose)
				data_rs.append(rTs)
				data_c.append(cPose)
				data_cs.append(cTs)
				data_z.append(zPose)
				data_zs.append(zTs)
 		else:
			data_list = zip(camera_poses, camera_ts, robot_poses, robot_ts)
			for cPose, cTs, rPose, rTs in data_list:
				rPose = numpy.reshape(rPose,(16,1))
				cPose = numpy.reshape(cPose,(16,1))
				data_r.append(rPose)
				data_rs.append(rTs)
				data_c.append(cPose)
				data_cs.append(cTs)
	
		fileRoot, fileExt = os.path.splitext(filename)
	
		matExt = '.mat'
		matFilename = fileRoot + '_' + arm + matExt
		print matFilename
		scipy.io.savemat(matFilename, {'r':data_r, 'rs':data_rs, 'c':data_c, 'cs':data_cs, 'z':data_z, 'zs':data_zs, 't':data_t})
