import roslib
roslib.load_manifest('raven_2_trajectory')
import rospy
from geometry_msgs.msg import *
from math import *
from raven_2_msgs.msg import *
from std_msgs.msg import Header
import copy
import sys, os.path
from math import *

import tf
import tfx

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

	def __call__(self,prompt = None,yesno=False):
		if prompt:
			import sys
			print prompt,
			sys.stdout.flush()
		ch = self.impl()
		if prompt:
			print ''
		if yesno:
			return ch.lower() == 'y'
	
	def yesno(self,prompt = None):
		return self(prompt=prompt,yesno=True)


class Stage(object):
	def __init__(self,name,duration,cb):
		self.name = name
		self.duration = rospy.Duration(duration)
		self.cb = cb
		self.is_pause = False
	
	def _pause(self,*args):
		start_time = rospy.Time.now()
		print 'press any key to continue'
		getch = _Getch()
		try:
			cmd = getch()
			if _Getch.is_ctrl_c(cmd):
				rospy.signal_shutdown('user terminated')
		except Exception, e:
			print 'exception:',e
		self.duration = rospy.Time.now() - start_time
	
	@staticmethod
	def PAUSE(name=None):
		if name is None:
			name = 'PAUSE'
		s = Stage(name,0,None)
		s.cb = s._pause
		s.is_pause = True
		return s
	
	@staticmethod
	def DELAY(duration,name=None):
		if name is None:
			name = 'DELAY'
		def fn(cmd, t): pass
		s = Stage(name,duration,fn)
		return s
	
	@staticmethod
	def stage_breaks(stages):
		stage_breaks = [rospy.Duration(0)]
		for stage in stages:
			stage_breaks.append(stage.duration + stage_breaks[-1])
		return stage_breaks

class TrajectoryPlayer(object):
	def __init__(self,tf_listener=None,arms=['R']):
		self.stages = []
		self.init_poses = {}
		self.start_time = None
		
		self.arms = arms
		
		self.default_speed = 0.01
		
		self.tf_listener = tf_listener
		if self.tf_listener is None:
			self.tf_listener = tfx.TransformListener.instance()
		rospy.loginfo('waiting for transform')
		for arm in arms:
			self.tf_listener.waitForTransform('/0_link','/tool_'+arm,rospy.Time(0),rospy.Duration(5))
		
		for arm in arms:
			self.init_poses[arm] = tfx.pose(tfx.lookupTransform('/0_link','/tool_'+arm))
			
		self.pub_cmd = rospy.Publisher('raven_command', RavenCommand)
		
		self.current_state = None
		self.current_poses = {}
		self.current_grasps = {}
		self.state_sub = rospy.Subscriber('raven_state',raven_2_msgs.msg.RavenState,self._state_callback)
	
		self.header = Header()
		self.header.frame_id = '/0_link'
	
	def _state_callback(self,msg):
		self.current_state = msg
		for arm in msg.arms:
			if arm.name in self.arms:
				self.current_poses[arm.name] = tfx.pose(arm.tool.pose,header=msg.header)
				self.current_grasps[arm.name] = arm.tool.grasp
	
	def _check(self,arm):
		if arm is None:
			if len(self.arms) == 1:
				return self.arms[0]
			else:
				raise Exception('Must specify arm!')
		else:
			return arm
	
	def add_stage(self,name,duration,cb):
		self.stages.append(Stage(name,duration,cb))
	
	def add_pause(self,name=None):
		self.stages.append(Stage.PAUSE(name=name))
	
	def add_delay(self,duration,name=None):
		self.stages.append(Stage.DELAY(duration, name=name))
	
	def add_pose_to_pose(self,name,start,end,arm=None,duration=None,speed=None):
		start = tfx.pose(start)
		end = tfx.pose(end)
		if duration is None:
			if speed is None:
				speed = self.default_speed
			duration = end.position.distance(start.position) / speed
		def fn(cmd,t):
			pose = start.interpolate(end,t)
			
			tool_pose = pose.msg.Pose()
		
			TrajectoryPlayer.add_arm_pose_cmd(cmd,self._check(arm),tool_pose)
		self.add_stage(name,duration,fn)
	
	def add_point_to_point(self,name,start_pos,end_pos,orientation,arm=None,duration=None,speed=None):
		start = tfx.pose(start_pos,orientation)
		end = tfx.pose(end_pos,orientation)
		self.add_pose_to_pose(name, start, end, arm=arm, duration=duration, speed=speed)
	
	def add_pose_to_point(self,name,start_pose,end_point,arm=None,duration=None,speed=None):
		start = tfx.pose(start_pose)
		end = tfx.pose(end_point,start.orientation)
		self.add_pose_to_pose(name, duration, start, end, arm=arm, duration=duration, speed=speed)
	
	def add_goto_first_pose(self,pose,arm=None,name=None,duration=None,speed=None):
		arm = self._check(arm)
		if name is None:
			name = 'Goto first pose '+arm
		start = self.init_poses[arm]
		end = tfx.pose(pose)
		intermediate = start.copy()
		intermediate.position.z = end.position.z
		dist1 = intermediate.position.distance(start.position)
		dist2 = end.position.distance(intermediate.position)
		if duration is not None:
			total_dist = dist1 + dist2
			dist1_frac = dist1 / total_dist
			dist2_frac = dist2 / total_dist
			duration1 = duration * dist1_frac
			duration2 = duration * dist2_frac
		else:
			if speed is None:
				speed = default_speed
			duration1 = dist1 / speed
			duration2 = dist2 / speed
		self.add_pose_to_pose(name + ' z',start,intermediate,arm=arm,duration=duration1)
		self.add_pose_to_pose(name + ' xy',intermediate,end,arm=arm,duration=duration2)
	
	def add_circle(self,plane,name,center,radius,orientation,num_circles=1,arm=None,duration_per_circle=True,duration=None,speed=None):
		if duration and duration_per_circle:
			duration = duration * num_circles
		if duration is None:
			if speed is None:
				speed = self.default_speed
			total_dist = 2 * pi * radius * num_circles
			duration = total_dist / speed
		center = tfx.point(center)
		orientation = tfx.quaternion(orientation)
		def fn(cmd,t):
			if plane == 'xy':
				pt = center + [radius*cos(num_circles*2.*pi*t),num_circles*radius*sin(num_circles*2.*pi*t),0]
			elif plane == 'xz':
				pt = center + [radius*cos(num_circles*2.*pi*t),0,num_circles*radius*sin(num_circles*2.*pi*t)]
			elif plane == 'yz':
				pt = center + [0,radius*cos(num_circles*2.*pi*t),num_circles*radius*sin(num_circles*2.*pi*t)]
			
			tool_pose = Pose()
			tool_pose.position = pt.msg.Point()
			tool_pose.orientation = orientation.msg.Quaternion()
		
			TrajectoryPlayer.add_arm_pose_cmd(cmd,self._check(arm),tool_pose)
		self.add_stage(name,duration,fn)
	
	def add_xy_circle(self,name,center,radius,orientation,num_circles=1,arm=None,duration_per_circle=True,duration=None,speed=None):
		self.add_circle('xy',name,center,radius,orientation,num_circles=num_circles,arm=arm,duration_per_circle=duration_per_circle,duration=duration,speed=speed)
	
	def add_open_gripper(self,duration=2,arm=None,name='Open gripper'):
		def fn(cmd,t):
			TrajectoryPlayer.add_arm_grasp_cmd(cmd, self._check(arm), grasp=1, grasp_option=ToolCommand.GRASP_INCREMENT_SIGN)
		self.add_stage(name,duration,fn)
	
	def add_close_gripper(self,duration=2,arm=None,name='Close gripper'):
		def fn(cmd,t):
			TrajectoryPlayer.add_arm_grasp_cmd(cmd, self._check(arm), grasp=-1, grasp_option=ToolCommand.GRASP_INCREMENT_SIGN)
		self.add_stage(name,duration,fn)

	def add_set_gripper(self,value,duration=2,arm=None,name=None):
		if name is None:
			name = 'Set gripper %.2f' % value
		def fn(cmd,t):
			TrajectoryPlayer.add_arm_grasp_cmd(cmd, self._check(arm), grasp=value, grasp_option=ToolCommand.GRASP_SET_NORMALIZED)
		self.add_stage(name,duration,fn)

	def play(self):
		rate = rospy.Rate(50)
		
		cmd = None
		
		while self.current_state is None and not rospy.is_shutdown():
			rate.sleep()
		
		if self.current_state.runlevel == 0:
			rospy.loginfo("Raven in E-STOP, waiting")
			while self.current_state.runlevel == 0 and not rospy.is_shutdown():
				rate.sleep()
		
		self.start_time = rospy.Time.now()
		
		success = None
		
		last_stage_ind = -1
		while not rospy.is_shutdown():
			if last_stage_ind != -1:
				sys.stdout.write("\r\x1b[K")
				sys.stdout.flush()
			
			if self.current_state.runlevel == 0:
				rospy.logerr('Raven in E-STOP, exiting')
				success = False
				break
			stage_breaks = Stage.stage_breaks(self.stages)
			now = rospy.Time.now()
			dur_from_start = now - self.start_time
			stage_ind = 0
			for idx,stage_break in enumerate(stage_breaks):
				if stage_break > dur_from_start:
					stage_ind = idx-1
					break
			else:
				rospy.loginfo("Finished!")
				success = True
				break
			stage_ind = min(stage_ind,last_stage_ind + 1)
				
			stage_changed = stage_ind != last_stage_ind
			last_stage_ind = stage_ind
			
			stage = self.stages[stage_ind]
			
			if stage.duration.is_zero():
				t = 1
			else:
				t = (dur_from_start - stage_breaks[stage_ind]).to_sec() / stage.duration.to_sec()
			
			if stage_changed:
				rospy.loginfo("Stage #%i/%i [%4.1fs] %s",stage_ind+1,len(self.stages),stage.duration.to_sec(),stage.name)
			else:
				sys.stdout.write("%.3f" % t)
				sys.stdout.flush()
			
			if stage.is_pause:
				stage.cb()
				continue
			
			self.header.stamp = now
			
			cmd = RavenCommand()
			cmd.header = self.header
			cmd.pedal_down = True
			
			stage.cb(cmd,t)
			
			if stage_changed:
				pass #print '\n\n' + str(cmd) + '\n\n'
			
			self.pub_cmd.publish(cmd)
			
			rate.sleep()
		return success
	
	
	@staticmethod
	def add_arm_cmd(cmd,arm_name,
				tool_pose=None,pose_option=ToolCommand.POSE_OFF,
				grasp=0,grasp_option=ToolCommand.GRASP_OFF):
		cmd.arm_names.append(arm_name)
			
		arm_cmd = ArmCommand()
		arm_cmd.active = True
		
		tool_cmd = ToolCommand()
		tool_cmd.pose_option = pose_option
		if tool_pose is not None:
			tool_cmd.pose = tool_pose
		
		tool_cmd.grasp_option = grasp_option
		tool_cmd.grasp = grasp
		
		arm_cmd.tool_command = tool_cmd
		
		cmd.arms.append(arm_cmd)
		
	@staticmethod
	def add_arm_pose_cmd(cmd,arm_name,tool_pose,pose_option=ToolCommand.POSE_ABSOLUTE):
		return TrajectoryPlayer.add_arm_cmd(cmd, arm_name, tool_pose=tool_pose, pose_option=pose_option, grasp_option=ToolCommand.GRASP_OFF)
	
	@staticmethod
	def add_arm_grasp_cmd(cmd,arm_name,grasp,grasp_option=ToolCommand.GRASP_INCREMENT_SIGN):
		return TrajectoryPlayer.add_arm_cmd(cmd, arm_name, grasp=grasp, grasp_option=grasp_option, pose_option=ToolCommand.POSE_OFF)