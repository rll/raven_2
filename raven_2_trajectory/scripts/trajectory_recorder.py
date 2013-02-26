#!/usr/bin/env python
import roslib; roslib.load_manifest("raven_2_trajectory")
import rospy
import rosbag
import math, time
import tf
from raven_2_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from tf.msg import tfMessage
import sys, os.path
import optparse

from raven_2_trajectory.srv import *
from optparse import OptionParser

ARMS = ['L','R'] #lr = ['L','R']
ACTIVE_ARMS = [True for arm in ARMS]

BASE_FRAME = '/0_link'
END_EFFECTOR_FRAME = 'tool_' + ARMS[0]

RAVEN_STATE_TOPIC = "raven_state"
RAVEN_ARRAY_STATE_TOPIC = RAVEN_STATE_TOPIC + "/array"

RAVEN_COMMAND_TOPIC = "raven_command";

COMMAND_POSE_TOPIC = "tool_pose/command";
COMMAND_POSE_ARM_TOPIC = COMMAND_POSE_TOPIC + "/%s";

TOOL_POSE_TOPIC = "tool_pose";
TOOL_POSE_ARM_TOPIC = TOOL_POSE_TOPIC + "/%s";

class TrajectoryRecorder:
	
	def __init__(self,listener,use_array_state=False,use_pose_arrays=False,service=True):
		self.listener = listener
		rospy.loginfo("waiting for transform from %s to %s",BASE_FRAME, END_EFFECTOR_FRAME)
		tries = 0
		while not rospy.is_shutdown():
			tries += 1
			#print "Try #%d" % tries
			try:
				self.listener.waitForTransform(BASE_FRAME, END_EFFECTOR_FRAME, rospy.Time(0), rospy.Duration(5.0))
				break
			except tf.Exception, e:
				continue
		rospy.loginfo("got it!")
		
		self.bag = None
		self.active = False
		self.dir = ''
		self.default_fileprefix = "traj"
		self.current_fileprefix = None
		
		self.raven_state = None
		self.raven_state_sub = rospy.Subscriber(RAVEN_STATE_TOPIC,RavenState,self.raven_state_callback)
		
		self.raven_array_state = None
		if use_array_state:
			self.raven_array_state_sub = rospy.Subscriber(RAVEN_ARRAY_STATE_TOPIC,RavenArrayState,self.raven_array_state_callback)
		
		self.joint_state = None
		self.joint_state_sub = rospy.Subscriber("joint_states",JointState,self.joint_callback)

		self.tf = None
		self.tf_sub = rospy.Subscriber("/tf",tfMessage,self.tf_callback)
		
		self.raven_command = None
		self.raven_command_sub = rospy.Subscriber(RAVEN_COMMAND_TOPIC,RavenCommand,self.raven_command_callback)
		
		self.raven_command_trajectory = None
		
		self.tool_pose_array = None
		self.tool_pose = {}
		
		if use_pose_arrays:
			self.tool_pose_array_sub = rospy.Subscriber(TOOL_POSE_TOPIC,PoseArray,self.tool_pose_array_callback)
		else:
			self.tool_pose_sub = {}
			for arm in ARMS:
				self.tool_pose_sub[arm] = rospy.Subscriber(TOOL_POSE_ARM_TOPIC % arm,PoseStamped,lambda msg,arm=arm: self.tool_pose_callback(msg,arm))

		self.cmd_pose_array = None
		self.cmd_pose = {}
		if use_pose_arrays:
			self.cmd_pose_array_sub = rospy.Subscriber(COMMAND_POSE_TOPIC,PoseArray,self.cmd_pose_array_callback)
		else:
			self.cmd_pose_sub = {}
			for arm in ARMS:
				self.cmd_pose_sub[arm] = rospy.Subscriber(COMMAND_POSE_ARM_TOPIC % arm,PoseStamped,lambda msg,arm=arm: self.cmd_pose_callback(msg,arm))

		self.cmd_pose_trajectory = None
		
		if service:
			self.service = rospy.Service("record_trajectory",RecordTrajectory,self.service_callback)

		rospy.loginfo("let's do this!")
	
	def getFilename(self,prefix):
		if not prefix:
			prefix = self.default_fileprefix
		self.current_fileprefix = prefix
		return os.path.join(self.dir,prefix + "_" + time.strftime("%Y_%m_%d_T%H_%M_%S") + ".bag")

	def service_callback(self,req):
		try:
			if req.activate:
				already_active = self.active
				if self.active and req.new_bag and self.bag:
					rospy.loginfo("Closing current bag %s",self.bag.filename)
					self.close()
				if not self.bag or (req.filename and req.filename != self.current_fileprefix):
					rospy.loginfo("Activating...")
					fname = self.getFilename(req.filename)
					self.bag = rosbag.Bag(fname,'w')
					rospy.loginfo("Writing to %s",self.bag.filename)
				self.active = True
				if not already_active:
					return RecordTrajectoryResponse(RecordTrajectoryResponse.SUCCESS)
				else:
					return RecordTrajectoryResponse(RecordTrajectoryResponse.NO_CHANGE)
			elif self.active:
				rospy.loginfo("Deactivating...")
				self.close()
				self.active = False
				self.current_fileprefix = None
				return RecordTrajectoryResponse(RecordTrajectoryResponse.SUCCESS)
			return RecordTrajectoryResponse(RecordTrajectoryResponse.NO_CHANGE)
		except Exception, e:
			rospy.logerr("Exception: %s",str(e))
			return RecordTrajectoryResponse(RecordTrajectoryResponse.ERROR)

	def joint_callback(self,msg):
		self.joint_state = msg

	def raven_state_callback(self,msg):
		self.raven_state = msg
	
	def raven_array_state_callback(self,msg):
		self.raven_array_state = msg

	def tf_callback(self,msg):
		self.tf = msg

	def raven_command_callback(self,msg):
		self.raven_command = msg
		if not self.active:
			return
		if not self.raven_command_trajectory:
			self.raven_command_trajectory = RavenTrajectoryCommand()
			self.raven_command_trajectory.header = msg.header
			self.raven_command_trajectory.controller = msg.controller
		pt = RavenTrajectoryCommandPoint()
		pt.arm_names = msg.arm_names
		pt.arms = msg.arms
		pt.time_from_start = msg.header.stamp - self.raven_command_trajectory.header.stamp
		self.raven_command_trajectory.commands.append(pt)
		

	def tool_pose_callback(self,msg,arm):
		self.tool_pose[arm] = msg
	
	def tool_pose_array_callback(self,msg):
		self.tool_pose_array = msg

	def cmd_pose_callback(self,msg,arm):
		armid = ARMS.index(arm)
		self.cmd_pose[arm] = msg
		if not self.active:
			return
		if not self.cmd_pose_trajectory:
			self.cmd_pose_trajectory = RavenTrajectoryCommand()
			self.cmd_pose_trajectory.header = msg.header
			self.cmd_pose_trajectory.controller = Constants.CONTROLLER_END_EFFECTOR
		time_from_start = msg.header.stamp - self.cmd_pose_trajectory.header.stamp
		ind = -1
		for i in reversed(xrange(len(self.cmd_pose_trajectory.commands))):
			valid = None
			if self.cmd_pose_trajectory.commands[i].time_from_start == time_from_start:
				ind = i
				break
			if self.cmd_pose_trajectory.commands[i].time_from_start < time_from_start:
				new_pt = RavenTrajectoryCommandPoint()
				new_pt.time_from_start = time_from_start
				new_pt.arm_names = [arm for arm in ARMS]
				new_pt.arms = [ArmCommand() for arm in ARMS]
				self.cmd_pose_trajectory.commands.append(new_pt)
				break
		else:
			new_pt = RavenTrajectoryCommandPoint()
			new_pt.time_from_start = time_from_start
			new_pt.arm_names = [arm for arm in ARMS]
			new_pt.arms = [ArmCommand() for arm in ARMS]
			self.cmd_pose_trajectory.commands.append(new_pt)
		
		self.cmd_pose_trajectory.commands[ind].arms[armid].active = True
		tool_cmd = ToolCommand()
		tool_cmd.relative = False
		tool_cmd.tool_pose = msg.pose
		tool_cmd.grasp_option = ToolCommand.GRASP_OFF
		self.cmd_pose_trajectory.commands[ind].arms[armid].tool_command = tool_cmd
	
	def cmd_pose_array_callback(self,msg):
		self.cmd_pose_array = msg;
				 

	def close(self):
		if self.bag is not None:
			self.active = False
			
			if self.raven_command_trajectory:
				self.bag.write(RAVEN_COMMAND_TOPIC + '/trajectory', self.raven_command_trajectory,self.raven_command_trajectory.header.stamp)
			if self.cmd_pose_trajectory:
				self.bag.write(COMMAND_POSE_TOPIC + '/trajectory', self.cmd_pose_trajectory,self.cmd_pose_trajectory.header.stamp)
			
			self.raven_command_trajectory = None
			
			self.cmd_pose_trajectory = None
			
			self.bag.close()
			rospy.loginfo("Closed bag %s",self.bag.filename)
			self.bag = None
	
	def write(self):
		if not self.active: return
		if not self.joint_state: return
		try:
			#print "writing..."
			self.bagwrite('joint_states',self.joint_state)
			self.bagwrite(RAVEN_STATE_TOPIC,self.raven_state)
			self.bagwrite(RAVEN_ARRAY_STATE_TOPIC,self.raven_array_state)
			self.bagwrite('/tf',self.tf)
			self.bagwrite(RAVEN_COMMAND_TOPIC, self.raven_command)
			self.bagwrite(TOOL_POSE_TOPIC, self.tool_pose_array)
			self.bagwrite(COMMAND_POSE_TOPIC, self.cmd_pose_array)
			for arm in ARMS:
				if self.tool_pose.has_key(arm):
					self.bagwrite(TOOL_POSE_ARM_TOPIC % arm,self.tool_pose[arm])
				if self.cmd_pose.has_key(arm):
					self.bagwrite(COMMAND_POSE_ARM_TOPIC % arm,self.cmd_pose[arm])
			now = self.joint_state.header.stamp
		except (tf.LookupException, tf.ConnectivityException), e:
			rospy.logerr("exception! %s",str(e))
	
	def bagwrite(self,topic,msg):
		if not msg: return
		if topic == '/tf' and msg.transforms:
			max_stamp = rospy.Time(0)
			for tf in msg.transforms:
				if tf.header.stamp > max_stamp:
					max_stamp = tf.header.stamp
		   	self.bag.write(topic, msg, max_stamp)
		else:
			self.bag.write(topic, msg, msg.header.stamp if msg._has_header else rospy.Time.now())

if __name__ == "__main__":
	rospy.init_node("raven_2_trajectory_recorder",anonymous=True)
	listener = tf.TransformListener()
	
	parser = OptionParser()
	
	parser.add_option('-d','--dir',help='Directory to save to')
	
	parser.add_option('-a','--use-arrays',help='Use array state and pose arrays',action='store_true',default=False)
	parser.add_option('--array-state',action='store_true',default=False)
	parser.add_option('--pose-arrays',action='store_true',default=False)
	
	parser.add_option('--now',action='store_true',default=False)
	
	(options,args) = parser.parse_args(rospy.myargv())
	
	tr = TrajectoryRecorder(listener, service= not options.now,
						use_array_state = options.array_state or options.use_arrays,
						use_pose_arrays = options.pose_arrays or options.use_arrays)
	
	if options.dir:
		tr.dir = options.dir

	if len(args) > 1:
		tr.default_fileprefix += args[-1]

	if options.now:
		req = RecordTrajectoryRequest(True,False,"")
		tr.service_callback(req)

	try:
		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			try:
				tr.write()
			except Exception, e:
				rospy.logerr("Exception while writing: %s",str(e))
				print e
			rate.sleep()
	finally:
		tr.close()
	
	
