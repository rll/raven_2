#!/usr/bin/env python

import roslib
roslib.load_manifest('raven_2_trajectory')
import rospy
from geometry_msgs.msg import *
from math import *
from raven_2_msgs.msg import *
from std_msgs.msg import Header
import copy
import sys,optparse, os.path

import tf
from tfx.canonical_tf import *
from tfx.tb_angles import tb_angles

pub_cmd = None
tf_listener = None
start_time = None
init_pose_left = None
init_pose_right = None

tool_orientation = canonical_rotation_tb(0, 90, 0)

approach_start = 0.06
approach_end = 0.00
retract_end = 0.14
retract_drop = 0.06

circle_duration = rospy.Duration(3)
circle_radius = 0.032 #0.027
circle_x_adj = 0.008
circle_around = 2*pi + pi

class _Getch:
	"""Gets a single character from standard input.  Does not echo to the screen."""
	@staticmethod
	def is_ctrl_c(ch):
		return ord(ch) == 3
	
	class _GetchWindows:
		def __init__(self):
			import msvcrt
	
		def __call__(self):
			import msvcrt
			return msvcrt.getch()
	class _GetchUnix:
		def __init__(self):
			import tty, sys
	
		def __call__(self):
			import sys, tty, termios
			fd = sys.stdin.fileno()
			old_settings = termios.tcgetattr(fd)
			try:
				tty.setraw(sys.stdin.fileno())
				ch = sys.stdin.read(1)
			finally:
				termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
			return ch
	
	def __init__(self):
		try:
			self.impl = _Getch._GetchWindows()
		except ImportError:
			self.impl = _Getch._GetchUnix()

	def __call__(self): return self.impl()


class Stage:
	def __init__(self,name,duration,cb):
		self.name = name
		self.duration = rospy.Duration(duration)
		self.cb = cb
		self.is_wait = False
	
	def _wait(self,*args):
		start_time = rospy.Time.now()
		print 'press any key to continue'
		getch = _Getch()
		try:
			cmd = getch().lower()
			if ord(cmd) == 3:
				rospy.signal_shutdown('user terminated')
		except Exception, e:
			print 'exception:',e
		self.duration = rospy.Time.now() - start_time
	
	@staticmethod
	def WAIT(name='WAIT'):
		 s = Stage(name,0,None)
		 s.cb = s._wait
		 s.is_wait = True
		 return s
	
	@staticmethod
	def stage_breaks(stages):
		stage_breaks = [rospy.Duration(0)]
		for stage in stages:
			stage_breaks.append(stage.duration + stage_breaks[-1])
		return stage_breaks	

def add_arm_cmd(cmd,arm_name,tool_pose,pose_option=ToolCommand.POSE_ABSOLUTE,grasp_option=ToolCommand.GRASP_OFF,grasp=0):
	cmd.arm_names.append(arm_name)
		
	arm_cmd = ArmCommand()
	arm_cmd.active = True
	
	tool_cmd = ToolCommand()
	tool_cmd.pose_option = pose_option
	tool_cmd.pose = tool_pose
	
	tool_cmd.grasp_option = ToolCommand.GRASP_INCREMENT_SIGN #grasp_option
	tool_cmd.grasp = 0 #grasp
	
	arm_cmd.tool_command = tool_cmd
	
	cmd.arms.append(arm_cmd)


class Foo:
	def __init__(self,height,x_dim,y_dim):
		#self.init_pos = canonical_point(-0.108616,-0.02029,-0.1408) #canonical_point(-0.088616,-0.02029,-0.11908)
		#self.init_pos = canonical_point(-0.148616,-0.02029,-0.1408)
		self.init_pos = canonical_point(-0.148616,0.03029,-0.1408)
		self.height = height
		self.x_dim = x_dim
		self.y_dim = y_dim
		
		self.first_pos = canonical_copy(self.init_pos)
		self.first_pos[2] = height
		self.first_pos[2] -= 0.0018
		
		self.second_pos = canonical_copy(self.first_pos)
		self.second_pos[0] += x_dim
		self.second_pos[2] -= 0.0018
		
		self.third_pos = canonical_copy(self.second_pos)
		self.third_pos[1] -= y_dim
		#self.third_pos[2] -= 0.0005
		
		self.fourth_pos = canonical_copy(self.third_pos)
		self.fourth_pos[0] -= x_dim
		
		self.last_pos = canonical_copy(self.init_pos)
		self.last_pos[2] = height
		
	
	def move_pos(self,cmd,t,start_pos,end_pos,orientation):
		init_pos = start_pos
		end_pos = end_pos
		
		right_tool_pose = Pose()
		pt = canonical_point((end_pos - init_pos)*t + init_pos,copy=True)
		right_tool_pose.position = pt.msg.Point()
		right_tool_pose.orientation = orientation.msg.Quaternion()
		
		add_arm_cmd(cmd,'R',right_tool_pose)
	
	def goto_init_pose(self,cmd,t):
		init_pos = init_pose_right.position
		#end_pos = canonical_point(-0.088616,-0.02029,-0.11908)
		end_pos = self.init_pos
		
		right_tool_pose = Pose()
		pt = canonical_point((end_pos - init_pos)*t + init_pos,copy=True)
		right_tool_pose.position = pt.msg.Point()
		right_tool_pose.orientation = tool_orientation.slerp(init_pose_right.orientation,t).msg.Quaternion()
		
		add_arm_cmd(cmd,'R',right_tool_pose)
	
	def close_gripper(self,cmd,t):
		cmd.arm_names.append('R')
			
		arm_cmd = ArmCommand()
		arm_cmd.active = True
		
		tool_cmd = ToolCommand()
		tool_cmd.pose_option = ToolCommand.POSE_OFF
		
		tool_cmd.grasp_option = ToolCommand.GRASP_INCREMENT_SIGN
		tool_cmd.grasp = -1
		
		arm_cmd.tool_command = tool_cmd
		
		cmd.arms.append(arm_cmd)
	
	
	def lower_to_table(self,cmd,t):
		init_pos = self.init_pos
		end_pos = self.first_pos
		
		right_tool_pose = Pose()
		pt = canonical_point((end_pos - init_pos)*t + init_pos,copy=True)
		right_tool_pose.position = pt.msg.Point()
		right_tool_pose.orientation = tool_orientation.msg.Quaternion()
		
		add_arm_cmd(cmd,'R',right_tool_pose)
	
	def leg1(self,cmd,t):
		init_pos = self.first_pos
		end_pos = self.second_pos
		
		right_tool_pose = Pose()
		pt = canonical_point((end_pos - init_pos)*t + init_pos,copy=True)
		right_tool_pose.position = pt.msg.Point()
		right_tool_pose.orientation = tool_orientation.msg.Quaternion()
		
		add_arm_cmd(cmd,'R',right_tool_pose)
	
	def leg2(self,cmd,t):
		init_pos = self.second_pos
		end_pos = self.third_pos
		
		right_tool_pose = Pose()
		pt = canonical_point((end_pos - init_pos)*t + init_pos,copy=True)
		right_tool_pose.position = pt.msg.Point()
		right_tool_pose.orientation = tool_orientation.msg.Quaternion()
		
		add_arm_cmd(cmd,'R',right_tool_pose)
	
	def leg3(self,cmd,t):
		init_pos = self.third_pos
		end_pos = self.fourth_pos
		
		right_tool_pose = Pose()
		pt = canonical_point((end_pos - init_pos)*t + init_pos,copy=True)
		right_tool_pose.position = pt.msg.Point()
		right_tool_pose.orientation = tool_orientation.msg.Quaternion()
		
		add_arm_cmd(cmd,'R',right_tool_pose)
	
	def leg4(self,cmd,t):
		init_pos = self.fourth_pos
		end_pos = self.last_pos
		
		right_tool_pose = Pose()
		pt = canonical_point((end_pos - init_pos)*t + init_pos,copy=True)
		right_tool_pose.position = pt.msg.Point()
		right_tool_pose.orientation = tool_orientation.msg.Quaternion()
		
		add_arm_cmd(cmd,'R',right_tool_pose)
	
	def initial_l_pose(self,cmd,t):
		r_pose = canonical_tf(tf_listener.lookupTransform('/0_link','/tool_R',rospy.Time(0)))
		
		end_pos1 = init_pose_left.position
		end_pos1.x = r_pose.position.x + approach_start
		end_pos2 = canonical_point(end_pos1,copy=True)
		end_pos2.y = r_pose.position.y
		end_pos2.z = r_pose.position.z - circle_radius
		
		
		init_pose = Pose()
		init_pose.orientation = tool_orientation.msg.Quaternion()
		
		change_over = 0.7
		if t < change_over:
			start_pt = init_pose_left.position
			end_pt = end_pos1
			t = t / change_over
		else:
			start_pt = end_pos1
			end_pt = end_pos2
			t = (t-change_over)/(1-change_over)
		pt = canonical_point((end_pt - start_pt)*t + start_pt,copy=True)
		init_pose.position = pt.msg.Point()
		
		
		#add_arm_cmd(cmd,'L',init_pose)
		cmd.arm_names.append('L')
			
		arm_cmd = ArmCommand()
		arm_cmd.active = True
		
		tool_cmd = ToolCommand()
		tool_cmd.pose_option = ToolCommand.POSE_ABSOLUTE
		tool_cmd.pose = init_pose
		
		tool_cmd.grasp_option = ToolCommand.GRASP_OFF
		
		arm_cmd.tool_command = tool_cmd
		
		cmd.arms.append(arm_cmd)
	
	def set_l_gripper(self,cmd,t):
		cmd.arm_names.append('L')
			
		arm_cmd = ArmCommand()
		arm_cmd.active = True
		
		tool_cmd = ToolCommand()
		tool_cmd.pose_option = ToolCommand.POSE_OFF
		
		tool_cmd.grasp_option = ToolCommand.GRASP_INCREMENT_SIGN
		tool_cmd.grasp = -1
		
		arm_cmd.tool_command = tool_cmd
		
		cmd.arms.append(arm_cmd)
	
	def approach(self,cmd,t):
		r_pose = canonical_tf(tf_listener.lookupTransform('/0_link','/tool_R',rospy.Time(0)))
		
		pose = Pose()
		pose.orientation = left_tool_orientation.msg.Quaternion()
		
		pose.position = r_pose.position.msg.Point()
		
		pose.position.x = r_pose.position.x + (approach_end - approach_start) * t + approach_start
		pose.position.z = r_pose.position.z - circle_radius
		
		add_arm_cmd(cmd,'L',pose)
	
	def circle(self,cmd,t):
		r_pose = canonical_tf(tf_listener.lookupTransform('/0_link','/tool_R',rospy.Time(0)))
		
		pose = Pose()
		pose.orientation = left_tool_orientation.msg.Quaternion()
		
		pose.position = r_pose.position.msg.Point()
		
		pose.position.x = r_pose.position.x + approach_end + circle_x_adj * cos(circle_around * t)
		pose.position.y = r_pose.position.y - circle_radius * sin(circle_around * t)
		pose.position.z = r_pose.position.z - circle_radius * cos(circle_around * t)
		
		self.circle_end_pose = canonical_tf(pose)
		
		add_arm_cmd(cmd,'L',pose)
	
	def retract(self,cmd,t):
		r_pose = init_pose_right #canonical_tf(tf_listener.lookupTransform('/0_link','/tool_R',rospy.Time(0)))
		
		pose = Pose()
		pose.orientation = left_tool_orientation.msg.Quaternion()
		
#		end_pt = CanonicalPoint()
#		end_pt.x = r_pose.position.x + approach_end + circle_x_adj * cos(circle_around) + (retract_end - approach_end) * t
#		end_pt.y = r_pose.position.y - circle_radius * sin(circle_around)
#		end_pt.z = r_pose.position.z - circle_radius * cos(circle_around)

		end_pt = self.circle_end_pose.position
		end_pt.x = end_pt.x + retract_end * t
		end_pt.z = end_pt.z - retract_drop * t
		
		pose.position = end_pt.msg.Point()
		
		add_arm_cmd(cmd,'L',pose)

if __name__ == '__main__':
	rospy.init_node(os.path.basename(sys.argv[0][:-3]),anonymous=True)
	
	parser = optparse.OptionParser()
	
	parser.add_option('-t',type='float')
	
	(options,args) = parser.parse_args(rospy.myargv())
	
	x_dim = float(args[1])
	if len(args) > 2:
		y_dim = float(args[2])
	else:
		y_dim = x_dim
	
	tf_listener = tf.TransformListener()
	rospy.loginfo('waiting for transform')
	tf_listener.waitForTransform('/0_link','/tool_R',rospy.Time(0),rospy.Duration(5))
	rospy.loginfo('got it')
	
	#init_pose_left = canonical_tf(tf_listener.lookupTransform('/0_link','/tool_L',rospy.Time(0)))
	init_pose_right = canonical_tf(tf_listener.lookupTransform('/0_link','/tool_R',rospy.Time(0)))
	
	pub_cmd = rospy.Publisher('/raven_command', RavenCommand)
	
	header = Header()
	header.frame_id = '/0_link'
	
	#height = float(raw_input('Enter table height'))
	#height = -0.227
	height = -0.187
	
	foo = Foo(height=height,x_dim=x_dim,y_dim=y_dim)
	
	stages = []
	stages.append(Stage("Goto init Pose",3,foo.goto_init_pose))
	stages.append(Stage("Close gripper",2,foo.close_gripper))
	stages.append(Stage("Lower tool",2,foo.lower_to_table))
	
	leg_time = 4
	
	stages.append(Stage("Leg 1",leg_time,foo.leg1))
	stages.append(Stage("Leg 2",leg_time,foo.leg2))
	stages.append(Stage("Leg 3",leg_time,foo.leg3))
	stages.append(Stage("Leg 4",leg_time,foo.leg4))
	
	#stages.append(Stage("Initial L pose",4,foo.initial_l_pose))
	#stages.append(Stage("Set L Gripper",2,foo.set_l_gripper))
	#stages.append(Stage("Approach",2,foo.approach))
	#stages.append(Stage("Circle",circle_duration.to_sec(),foo.circle))
	#stages.append(Stage.WAIT("wait for user to grab string"))
	#stages.append(Stage("Retract",1,foo.retract))
	
	start_time = rospy.Time.now()
	
	rate = rospy.Rate(50)
	
	cmd = None
	
	last_stage_ind = -1
	while not rospy.is_shutdown():
		stage_breaks = Stage.stage_breaks(stages)
		now = rospy.Time.now()
		dur_from_start = now - start_time
		stage_ind = 0
		for idx,stage_break in enumerate(stage_breaks):
			if stage_break > dur_from_start:
				stage_ind = idx-1
				break
		else:
			rospy.loginfo("Finished!")
			break
		stage_ind = min(stage_ind,last_stage_ind + 1)
			
		stage_changed = stage_ind != last_stage_ind
		last_stage_ind = stage_ind
		
		stage = stages[stage_ind]
		
		if stage.duration.is_zero():
			t = 1
		else:
			t = (dur_from_start - stage_breaks[stage_ind]).to_sec() / stage.duration.to_sec()
		
		if stage_changed:
			sys.stdout.write('\n')
			sys.stdout.flush()
			#print cmd
			rospy.loginfo("Entering stage #%i/%i %s (%fs)",stage_ind+1,len(stages),stage.name,stage.duration.to_sec())
			#print dur_from_start.to_sec(), [b.to_sec() for b in stage_breaks]
		else:
			sys.stdout.write("\r\x1b[K"+str(t))
			sys.stdout.flush()
		
		if stage.is_wait:
			stage.cb()
			continue
		
		header.stamp = now
		
		cmd = RavenCommand()
		cmd.header = header
		cmd.pedal_down = True
		
		stage.cb(cmd,t)
		
		if stage_changed:
			pass #print '\n\n' + str(cmd) + '\n\n'
		
		pub_cmd.publish(cmd)
		
		rate.sleep()