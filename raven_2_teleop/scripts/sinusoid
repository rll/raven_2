#!/usr/bin/env python

import roslib
roslib.load_manifest('raven_2_trajectory')

import rospy
from raven_2_msgs.msg import *

from math import *
import sys
import optparse

class JointCommander:
	def __init__(self,arm_name,joint,amplitude=None,period=None):
		self.state = None
		self.first_state = None
		self.first_joint_value = {}
		self.arm_names = ['R','L']#[arm_name]
		self.selected_arm = arm_name
		
		self.joint = joint
		
		if not amplitude:
			amplitude = 5 * (pi / 180)
		self.amplitude = amplitude
		
		if not period:
			period = 10
		self.period = period
		
		
		self.start_time = None
		
		self.pub = rospy.Publisher('raven_command', RavenCommand)
		self.sub = rospy.Subscriber('raven_state',RavenState,self.state_cb)
		
	def state_cb(self,msg):
		if not self.first_state:
			self.first_state = msg
			for arm_name in self.arm_names:
				arm_state = [arm_state for arm_state in msg.arms if arm_state.name == arm_name][0]
				self.first_joint_value[arm_name] = arm_state.joints[self.joint].position
		self.state = msg
	
	def set_arm_to_current_state(self,arm_name,arm_cmd):
		arm_state = [arm_state for arm_state in self.state.arms if arm_state.name == arm_name][0]
		arm_cmd.tool_command.pose = arm_state.tool.pose
		arm_cmd.tool_command.grasp = arm_state.tool.grasp
		for joint_state in arm_state.joints:
		 	if joint_state.type == Constants.JOINT_TYPE_YAW or joint_state.type == Constants.JOINT_TYPE_GRASP:
		 		continue
		 	
			arm_cmd.joint_types.append(joint_state.type)
			joint_cmd = JointCommand()
			joint_cmd.command_type = JointCommand.COMMAND_TYPE_POSITION
			joint_cmd.value = joint_state.position
			arm_cmd.joint_commands.append(joint_cmd)
		#print arm_cmd
	
	def send_cmd(self):
		if not self.state:
			return
		now = rospy.Time.now()
		if not self.start_time:
			self.start_time = now
		
		dur_from_start = (now - self.start_time).to_sec()
		
		cmd = RavenCommand()
		cmd.header.stamp = now
		
		cmd.controller = Constants.CONTROLLER_MOTOR
		cmd.pedal_down = True
		
		for arm_name in self.arm_names:
			 cmd.arm_names.append(arm_name)
			 arm_cmd = ArmCommand()
			 self.set_arm_to_current_state(arm_name, arm_cmd)
			 arm_cmd.active = True
			 arm_cmd.tool_command.pose_option = ToolCommand.POSE_OFF
			 arm_cmd.tool_command.grasp_option = ToolCommand.GRASP_OFF
			 
			 #print arm_cmd
			 if arm_name == self.selected_arm:
			 	arm_cmd.joint_commands[self.joint].value = self.amplitude * sin(2 * pi * (dur_from_start/self.period)) + self.first_joint_value[arm_name]
			 	
			 cmd.arms.append(arm_cmd)
		
		#print cmd
		self.pub.publish(cmd)
		#rospy.sleep(30)

if __name__ == '__main__':
	rospy.init_node('sinusoid',anonymous=True)
	
	parser = optparse.OptionParser()
	
	(options,args) = parser.parse_args(rospy.myargv())
	
	arm_name = args[1]
	joint = int(args[2])
	
	amplitude = None
	if len(args) > 3:
		amplitude = float(args[3]) * (pi / 180)
	
	period = None
	if len(args) > 4:
		period = float(args[4])
		
	jc = JointCommander(arm_name, joint, amplitude=amplitude, period=period)
	
	
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		jc.send_cmd()
		
		rate.sleep()