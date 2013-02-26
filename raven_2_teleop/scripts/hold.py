#!/usr/bin/env python

import roslib
roslib.load_manifest('raven_2_trajectory')

import rospy
from raven_2_msgs.msg import *

class HoldPosition:
	def __init__(self,arms):
		self.state = None
		self.pub = rospy.Publisher('raven_command', RavenCommand)
		self.arms = arms
		
		self.sub = rospy.Subscriber('raven_state',RavenState,self.state_cb)
		
	def state_cb(self,msg):
		if self.state:
			return
		self.state = msg
		
	def send_cmd(self):
		if not self.state:
			return
		cmd = RavenCommand()
		cmd.header.stamp = rospy.Time.now()
		cmd.controller = Constants.CONTROLLER_JOINT_POSITION
		cmd.pedal_down = True
		for arm in self.arms:
			try:
				arm_state = [arm_state for arm_state in self.state.arms if arm_state.name == arm][0]
			except:
				continue
			cmd.arm_names.append(arm_state.name)
			arm_cmd = ArmCommand()
			arm_cmd.active = True
			arm_cmd.tool_command.pose_option = ToolCommand.POSE_OFF
			arm_cmd.tool_command.grasp_option = ToolCommand.GRASP_OFF
			for joint_state in arm_state.joints:
				if joint_state.type == Constants.JOINT_TYPE_YAW or joint_state.type == Constants.JOINT_TYPE_GRASP:
					continue
				arm_cmd.joint_types.append(joint_state.type)
				joint_cmd = JointCommand()
				joint_cmd.command_type = JointCommand.COMMAND_TYPE_POSITION
				joint_cmd.value = joint_state.position
				arm_cmd.joint_commands.append(joint_cmd)
			cmd.arms.append(arm_cmd)
		
		#print cmd
		self.pub.publish(cmd)
		#rospy.sleep(30)

if __name__ == '__main__':
	rospy.init_node('teleop_hold',anonymous=True)
	
	hp = HoldPosition(['L','R'])
	
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		hp.send_cmd()
		
		rate.sleep()