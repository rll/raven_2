#!/usr/bin/env python

import roslib
roslib.load_manifest('raven_2_trajectory')
import rospy
import rosbag
import numpy
import pickle
import os, os.path, sys, optparse
import shutil

from raven_2_trajectory.trajectory_io import extract_trajectory_from_bag

if __name__ == '__main__':
	rospy.init_node('extract_trajectory_array',anonymous=True)
	
	parser = optparse.OptionParser()
	
	parser.add_option('-p','--package',action='store_true',default=False)
	parser.add_option('-f','--force',action='store_true',default=False)
	
	(options,args) = parser.parse_args(args=rospy.myargv())
	
	if options.package or len(args) < 2:
		if options.package:
			dir = roslib.packages.get_pkg_subdir('raven_2_trajectory', 'trajectories', required=True)
		else:
			dir = '.'
		files = []
		bags = [os.path.join(dir,file) for file in os.listdir(dir) if file.endswith('.bag')]
		for bag in bags:
			pkl = bag[:-4] + '.pkl'
			if options.force or not os.path.isfile(pkl) or os.stat(bag).st_mtime - os.stat(pkl).st_mtime > 1:
				files.append(bag)
	else:		
		files = args[1:]
	
	for filename in files:
		extract_trajectory_from_bag(filename,save_pickle=True)
