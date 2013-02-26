import roslib; roslib.load_manifest('raven_2_trajectory')
import rosbag
from tfx.canonical_tf import *
import numpy
from raven_2_msgs.msg import *
from collections import namedtuple
import pickle
import shutil
import tf_conversions.posemath as pm

from raven_2_trajectory import TrajectoryArrayData

def extract_trajectory_from_bag(filename,save_pickle=False):
	print 'Extracting %s' % filename
	bag = rosbag.Bag(filename)
	
	arm_names = None
	arm_types = None
	joint_arm_inds = None
	joint_types = None
	
	times = None
	
	poses = None
	grasps = None
	
	pos = None
	vel = None
	
	mpos = None
	mvel = None
	torques = None
	
	pos_d = None
	vel_d = None
	
	mpos_d = None
	mvel_d = None
	
	for topic,msg,t in bag.read_messages(topics=['raven_state/array']):
		if rospy.is_shutdown():
			break
		if pos is None:
			arm_names = msg.arm_names
			arm_types = numpy.array([ord(i) for i in list(msg.arm_types)],dtype=int)
			joint_arm_inds = numpy.array([ord(i) for i in list(msg.joint_arm_inds)],dtype=int)
			joint_types = numpy.array(msg.joint_types,dtype=int)
			
			times = [msg.header.stamp.to_sec()]
			
			try:
				poses = [tuple([pm.toMatrix(pm.fromMsg(pose)) for pose in msg.tool.poses])]
				grasps = numpy.array(msg.tool.grasps)
			except:
				#old msg
				poses = [tuple([pm.toMatrix(pm.fromMsg(pose)) for pose in msg.tool_poses])]
				grasps = numpy.array(msg.grasps)
			
			pos = numpy.array(msg.joint_positions)
			vel = numpy.array(msg.joint_velocities)
			
			mpos = numpy.array(msg.motor_positions)
			mvel = numpy.array(msg.motor_velocities)
			torques = numpy.array(msg.torques)
			
			pos_d = numpy.array(msg.set_points.joint_positions)
			vel_d = numpy.array(msg.set_points.joint_velocities)
			
			mpos_d = numpy.array(msg.set_points.motor_positions)
			mvel_d = numpy.array(msg.set_points.motor_velocities)
		else:
			times.append(msg.header.stamp.to_sec())
			
			try:
				poses.append(tuple([pm.toMatrix(pm.fromMsg(pose)) for pose in msg.tool.poses]))
				grasps = numpy.vstack((grasps,numpy.array(msg.tool.grasps)))
			except:
				poses.append(tuple([pm.toMatrix(pm.fromMsg(pose)) for pose in msg.tool_poses]))
				grasps = numpy.vstack((grasps,numpy.array(msg.grasps)))
			
			pos = numpy.vstack((pos,numpy.array(msg.joint_positions)))
			vel = numpy.vstack((vel,numpy.array(msg.joint_velocities)))
			
			mpos = numpy.vstack((mpos,numpy.array(msg.motor_positions)))
			mvel = numpy.vstack((mvel,numpy.array(msg.motor_velocities)))
			torques = numpy.vstack((torques,numpy.array(msg.torques)))
			
			pos_d = numpy.vstack((pos_d,numpy.array(msg.set_points.joint_positions)))
			vel_d = numpy.vstack((vel_d,numpy.array(msg.set_points.joint_velocities)))
			
			mpos_d = numpy.vstack((mpos_d,numpy.array(msg.set_points.motor_positions)))
			mvel_d = numpy.vstack((mvel_d,numpy.array(msg.set_points.motor_velocities)))
		
		
	
	times = numpy.array(times)
	times = times - times[0]
	
	if save_pickle:
		pkld = {'arm_names':arm_names,'arm_types':arm_types,
			'joint_arm_inds':joint_arm_inds,'joint_types':joint_types,
			'times':times,
			'poses':poses, 'grasps':grasps,
			'pos':pos, 'vel':vel, 'mpos':mpos, 'mvel':mvel, 'torques':torques,
			'pos_d':pos_d, 'vel_d':vel_d, 'mpos_d':mpos_d, 'mvel_d':mvel_d}
		
		pickle_filename = filename[:-4] + '.pkl'
		
		pf = open(pickle_filename,'w')
		pickle.dump(pkld,pf)
		pf.close()
		shutil.copystat(filename, pickle_filename)
	
	return TrajectoryArrayData(arm_names,arm_types,
							joint_arm_inds,joint_types,
							times,
							poses,grasps,
							pos,vel,mpos,mvel,torques,pos_d,vel_d,mpos_d,mvel_d)

def load_trajectory(filename):
	if not filename.endswith('pkl'):
		if not filename.endswith('.'):
			filename += '.'
		filename += 'pkl'
	pf = open(filename)
	pkld = pickle.load(pf)
	pf.close()
	
	return TrajectoryArrayData(pkld['arm_names'],pkld['arm_types'],
							pkld['joint_arm_inds'],pkld['joint_types'],
							pkld['times'],
							pkld['poses'], pkld['grasps'],
							pkld['pos'],pkld['vel'],pkld['mpos'],pkld['mvel'],pkld['torques'],
							pkld['pos_d'],pkld['vel_d'],pkld['mpos_d'],pkld['mvel_d'])
