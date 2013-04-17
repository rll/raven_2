import roslib; roslib.load_manifest('raven_2_trajectory')
import rospy
import tfx
import numpy as np
import pandas as pd

from raven_2_msgs.msg import *

from trajectory import Trajectory

import datetime

from rosbag import Bag

_is_msg_type = lambda obj,typename: isinstance(obj,typename) or (hasattr(obj,'_type') and obj._type == typename._type)

def load_trajectory_from_bag(file,fast=True,fast_only=False):
	traj = []
	fast_traj = []
	
	bag = Bag(file)
	
	normal_topics = ['raven_state', 'raven_state/array']
	normal_topics = normal_topics + ['/' + topic for topic in normal_topics]
	
	fast_topic = ['raven_state/1000Hz']
	#fast_topic = fast_topic + ['/' + topic for topic in fast_topic]
	
	if fast_only:
		for topic, msg, t in bag.read_messages(topics=fast_topic):
			fast_traj.append(msg)
		return get_trajectory(fast_traj)
	
	for topic, msg, t in bag.read_messages(topics=normal_topics):
		traj.append(msg)
	
	if fast:
		for topic, msg, t in bag.read_messages(topics=fast_topic):
			fast_traj.append(msg)
	
	if fast_traj:
		return get_trajectory(fast_traj,traj)
	else:
		return get_trajectory(traj)

def get_trajectory(traj,traj2=None):
	robot_traj, arm_traj, joint_traj = get_trajectory_data(traj)
	if traj2:
		robot_traj2, arm_traj2, joint_traj2 = get_trajectory_data(traj2)
		
		robot_traj = robot_traj.combine_first(robot_traj2)
		arm_traj = arm_traj.combine_first(arm_traj2)
		joint_traj = joint_traj.combine_first(joint_traj2)
		
	return Trajectory(robot_traj, arm_traj, joint_traj)
	

def get_trajectory_data(traj):
	if _is_msg_type(traj,Raven1000HzStateArray):
		traj_new = []
		for state in traj.states:
			state_new = Raven1000HzStateArray()
			state_new.arm_info = traj.arm_info
			state_new.states = [state]
			traj_new.append(state_new)
		traj = traj_new
	elif _is_msg_type(traj[0],Raven1000HzStateArray):
		#traj = [state for state_array in traj for state in state_array.states]
		traj_new = []
		for state_array in traj:
			for state in state_array.states:
				state_new = Raven1000HzStateArray()
				state_new.arm_info = state_array.arm_info
				state_new.states = [state]
				traj_new.append(state_new)
		traj = traj_new
	
	robot_traj_data = {}
	arm_traj_data = {}
	joint_traj_data = {}
	
	for traj_pt in traj:
		robot_series, arm_series, joint_series = get_trajectory_point(traj_pt)
		robot_traj_data[robot_series.name] = robot_series
		arm_traj_data[arm_series.name] = arm_series
		joint_traj_data[joint_series.name] = joint_series
	
	robot_traj = pd.DataFrame(robot_traj_data).T
	arm_traj = pd.DataFrame(arm_traj_data).T
	joint_traj = pd.DataFrame(joint_traj_data).T
	
	return robot_traj, arm_traj, joint_traj
		
	
	

def get_trajectory_point(msg):
	robot_index = []
	robot_data = []
	def robot_add(key,value):
		robot_index.append(key)
		robot_data.append(value)
	
	arm_index = []
	arm_data = []
	def arm_add(arm,key1,key2,value):
		arm_index.append((arm,key1,key2))
		arm_data.append(value)
	
	joint_index = []
	joint_data = []
	def joint_add(arm,joint,key,value):
		joint_index.append((arm,joint,key))
		joint_data.append(value)
	
	
	if _is_msg_type(msg,RavenState):
		stamp = datetime.datetime.fromtimestamp(msg.header.stamp.to_sec())
		robot_add('frame',msg.header.frame_id)
		robot_add('runlevel',msg.runlevel)
		robot_add('sublevel',msg.sublevel)
		robot_add('pedal_down',msg.pedal_down)
		robot_add('master',msg.master)
		robot_add('controller',msg.controller)
		for arm in msg.arms:
			arm_name = arm.name
			arm_type_strings = raven_2_msgs.msg.Constants.ARM_TYPE_STRINGS.split(',')
			arm_add(arm_name,'info','name',arm_name)
			arm_add(arm_name,'info','type',arm_type_strings[arm.type])
			arm_add(arm_name,'info','tool_type',arm.tool_type)
			arm_add(arm_name,'base_pose','x',arm.base_pose.position.x)
			arm_add(arm_name,'base_pose','y',arm.base_pose.position.y)
			arm_add(arm_name,'base_pose','z',arm.base_pose.position.z)
			arm_add(arm_name,'base_pose','qx',arm.base_pose.orientation.x)
			arm_add(arm_name,'base_pose','qy',arm.base_pose.orientation.y)
			arm_add(arm_name,'base_pose','qz',arm.base_pose.orientation.z)
			arm_add(arm_name,'base_pose','qw',arm.base_pose.orientation.w)
			
			arm_add(arm_name,'tool_pose','x',arm.tool.pose.position.x)
			arm_add(arm_name,'tool_pose','y',arm.tool.pose.position.y)
			arm_add(arm_name,'tool_pose','z',arm.tool.pose.position.z)
			arm_add(arm_name,'tool_pose','qx',arm.tool.pose.orientation.x)
			arm_add(arm_name,'tool_pose','qy',arm.tool.pose.orientation.y)
			arm_add(arm_name,'tool_pose','qz',arm.tool.pose.orientation.z)
			arm_add(arm_name,'tool_pose','qw',arm.tool.pose.orientation.w)
			
			arm_add(arm_name,'tool_grasp','grasp',arm.tool.grasp)
			
			arm_add(arm_name,'tool_set_point_pose','x',arm.tool_set_point.pose.position.x)
			arm_add(arm_name,'tool_set_point_pose','y',arm.tool_set_point.pose.position.y)
			arm_add(arm_name,'tool_set_point_pose','z',arm.tool_set_point.pose.position.z)
			arm_add(arm_name,'tool_set_point_pose','qx',arm.tool_set_point.pose.orientation.x)
			arm_add(arm_name,'tool_set_point_pose','qy',arm.tool_set_point.pose.orientation.y)
			arm_add(arm_name,'tool_set_point_pose','qz',arm.tool_set_point.pose.orientation.z)
			arm_add(arm_name,'tool_set_point_pose','qw',arm.tool_set_point.pose.orientation.w)
			
			arm_add(arm_name,'tool_set_point_grasp','grasp',arm.tool_set_point.grasp)
			
			for idx, joint in enumerate(arm.joints):
				joint_type_strings = raven_2_msgs.msg.Constants.JOINT_TYPE_STRINGS.split(',')
				#joint_name = joint_type_strings[joint.type]
				joint_name = idx
				joint_add(arm_name,joint_name,'type',joint.type)
				joint_add(arm_name,joint_name,'state',joint.state)
				joint_add(arm_name,joint_name,'encoder_value',joint.encoder_value)
				joint_add(arm_name,joint_name,'encoder_offset',joint.encoder_offset)
				joint_add(arm_name,joint_name,'dac_command',joint.dac_command)
				joint_add(arm_name,joint_name,'position',joint.position)
				joint_add(arm_name,joint_name,'velocity',joint.velocity)
				joint_add(arm_name,joint_name,'motor_position',joint.motor_position)
				joint_add(arm_name,joint_name,'motor_velocity',joint.motor_velocity)
				joint_add(arm_name,joint_name,'torque',joint.torque)
				joint_add(arm_name,joint_name,'gravity_estimate',joint.gravity_estimate)
				joint_command_type_strings = raven_2_msgs.msg.JointCommand.COMMAND_TYPE_STRINGS.split(',')
				joint_add(arm_name,joint_name,'command_type',joint_command_type_strings[joint.command.command_type])
				joint_add(arm_name,joint_name,'command_value',joint.command.value)
				joint_add(arm_name,joint_name,'integrated_position_error',joint.integrated_position_error)
				joint_add(arm_name,joint_name,'motor_position_set_point',joint.set_point.motor_position)
				joint_add(arm_name,joint_name,'motor_velocity_set_point',joint.set_point.motor_velocity)
				joint_add(arm_name,joint_name,'position_set_point',joint.set_point.position)
				joint_add(arm_name,joint_name,'velocity_set_point',joint.set_point.velocity)
			
	elif _is_msg_type(msg,RavenArrayState):
		stamp = datetime.datetime.fromtimestamp(msg.header.stamp.to_sec())
		pass
	elif _is_msg_type(msg,Raven1000HzStateArray):
		arm_info = msg.arm_info
		
		msg = msg.states[0]
		stamp = datetime.datetime.fromtimestamp(msg.header.stamp.to_sec())
		
		robot_add('frame',msg.header.frame_id)
		robot_add('runlevel',msg.runlevel)
		robot_add('sublevel',msg.sublevel)
		robot_add('pedal_down',msg.pedal_down)
		for arm, info in zip(msg.arms,arm_info):
			arm_name = info.name
			arm_type_strings = raven_2_msgs.msg.Constants.ARM_TYPE_STRINGS.split(',')
			arm_add(arm_name,'info','name',arm_name)
			arm_add(arm_name,'info','type',arm_type_strings[info.type])
			
			for idx in xrange(len(arm.joint_positions)):
				joint_name = idx
				joint_add(arm_name,joint_name,'type',info.joint_types[idx])
				joint_add(arm_name,joint_name,'state',arm.joint_states[idx])
				joint_add(arm_name,joint_name,'encoder_value',arm.motor_encoder_values[idx])
				joint_add(arm_name,joint_name,'encoder_offset',arm.motor_encoder_offsets[idx])
				joint_add(arm_name,joint_name,'dac_command',arm.dac_commands[idx])
				joint_add(arm_name,joint_name,'position',arm.joint_positions[idx])
				joint_add(arm_name,joint_name,'velocity',arm.joint_velocities[idx])
				joint_add(arm_name,joint_name,'motor_position',arm.motor_positions[idx])
				joint_add(arm_name,joint_name,'motor_velocity',arm.motor_velocities[idx])
				joint_add(arm_name,joint_name,'torque',arm.torques[idx])
				joint_add(arm_name,joint_name,'gravity_estimate',arm.gravity_estimates[idx])
				joint_add(arm_name,joint_name,'integrated_position_error',arm.integrated_position_errors[idx])
				joint_add(arm_name,joint_name,'motor_position_set_point',arm.set_points.motor_positions[idx])
				joint_add(arm_name,joint_name,'motor_velocity_set_point',arm.set_points.motor_velocities[idx])
				joint_add(arm_name,joint_name,'position_set_point',arm.set_points.joint_positions[idx])
				joint_add(arm_name,joint_name,'velocity_set_point',arm.set_points.joint_velocities[idx])
	
	robot_series = pd.Series(robot_data,index=robot_index,name=stamp)
	
	arm_multi_index = pd.MultiIndex.from_tuples(arm_index,names=['arm','field','subfield'])
	arm_series = pd.Series(arm_data,index=arm_multi_index,name=stamp)
	
	joint_multi_index = pd.MultiIndex.from_tuples(joint_index,names=['arm','joint','field'])
	joint_series = pd.Series(joint_data,index=joint_multi_index,name=stamp)
	
	return robot_series, arm_series, joint_series