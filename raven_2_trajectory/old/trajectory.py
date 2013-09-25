import roslib; roslib.load_manifest('raven_2_trajectory')
import rospy
import tfx
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from raven_2_msgs.msg import *
from geometry_msgs.msg import Pose,PoseStamped

class Trajectory(object):
	
	def __init__(self,*args,**kwargs): #robot,arms,joints,name=None):
		if len(args) == 1:
			other = args[0]
			self.robot = other.robot
			self.arms = other.arms
			self.joints = other.joints
			self.name = other.name
			if kwargs.get('copy',False):
				self.robot = self.robot.copy()
				self.arms = self.arms.copy()
				self.joints = self.joints.copy()
		else:
			self.robot = args[0]
			self.arms = args[1]
			self.joints = args[2]
		
		self.name = kwargs.get('name',None)
	
	def tool_poses(self,msg=False,tf=None):
		if 'tool_pose' not in self.arms.columns.levels[1]:
			return None
		poses = {}
		pose_data = self.arms.xs('tool_pose',axis=1,level='field')
		arm_names = pose_data.columns.levels[0]
		
		if tf:
			tf = tfx.transform(tf,pose=False)
		
		for arm in arm_names:
			arm_pose_data = pose_data[arm]
			arm_poses = []
			stamps = []
			for i in xrange(len(arm_pose_data)):
				if np.isnan(arm_pose_data.ix[i,'x']):
					continue
				translation = arm_pose_data.ix[i,['x','y','z']]
				rotation = arm_pose_data.ix[i,['qx','qy','qz','qw']]
				
				if msg:
					pose = PoseStamped()
					pose.header.frame_id = frame=self.robot.ix[i,'frame']
					pose.header.stamp = tfx.stamp(arm_pose_data.index[i]).ros
					pose.pose.position.x = translation[0]
					pose.pose.position.y = translation[1]
					pose.pose.position.z = translation[2]
					pose.pose.orientation.x = rotation[0]
					pose.pose.orientation.y = rotation[1]
					pose.pose.orientation.z = rotation[2]
					pose.pose.orientation.w = rotation[3]
				else:
					pose = tfx.pose(translation,rotation,stamp=arm_pose_data.index[i],frame=self.robot.ix[i,'frame'],name=arm)
					if tf:
						pose = tf * pose
				arm_poses.append(pose)
				stamps.append(arm_pose_data.index[i])
			if not msg:
				arm_poses = pd.Series(arm_poses,index=stamps,name=arm)
			poses[arm] = arm_poses
		if not msg:
			return pd.DataFrame(poses)
		else:
			return poses
		
	
	def joint_field(self,field):
		diff_suffix = '_diff'
		if field.endswith(diff_suffix):
			field = field[:-len(diff_suffix)]
			return self.joint_field(field + '_set_point') - self.joint_field(field)
		else:
			return self.joints.xs(field,axis=1,level='field').fillna(method='pad')
	
	def joint_type_names(self):
		index = self.joints.index
		index = [(i - index[0]).total_seconds() for i in index]
		
		types = self.joints.xs('type',axis=1,level='field').ix[0,:]
		if len(index.levels[0]) == 1:
			return [Constants.JOINT_TYPE_STRINGS.split(',')[int(types[i])] for i in xrange(len(types))]
		else:
			return [types.index[i][0] +' '+ Constants.JOINT_TYPE_STRINGS.split(',')[int(types[i])] for i in xrange(len(types))]
	
	def plot_joint_field(self,field,prefix=None,units=None):
		if prefix is None and self.name is not None:
			prefix = self.name
		
		name = field
		if prefix:
			name = prefix + ': ' + name
		#plt.figure(name.__hash__() % 1000)
		plt.figure(name)
		plt.clf()
		
		
		data = self.joint_field(field)
		
		index = data.index
		index = [(i - index[0]).total_seconds() for i in index]
		
		for i in xrange(len(index)):
			types = self.joints.xs('type',axis=1,level='field').ix[i,:]
			if not any(pd.isnull(types)):
				break
		
		p = plt.plot(index,data)
#		lines = plt.gca().lines
#		for i in xrange(len(types)):
#			if types[i] > 7:
#				lines[i].set_linestyle('..')
		
		plt.legend([types.index[i][0] +' '+ Constants.JOINT_TYPE_STRINGS.split(',')[int(types[i])] for i in xrange(len(types))],loc=2)
		plt.title(name)
		plt.xlabel('Time (s)')
		
		ylabel = field
		if units is not None:
			ylabel += ' (' + units + ')'
		plt.ylabel(ylabel)
		
		return p
	
	def plot_all_joint_fields(self,prefix=None):
		fields = {'position':'rad','velocity':'rad/s','motor_position':'rad/s','motor_velocity':'rad/s','torque':None,'position_set_point':'rad','position_diff':'rad'}
		
		plots = []
		for field,units in fields.iteritems():
			p = self.plot_joint_field(field,prefix=prefix,units=units)
			plots.append(p)
		
		p = self.plot_joint_field('state',prefix)
		plt.gca().set_yticklabels(JointState.STATE_STRINGS.split(','))
		plots.append(p)
		
		plt.draw()
		
		return plots