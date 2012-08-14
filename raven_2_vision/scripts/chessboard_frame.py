#!/usr/bin/env python
import roslib
import readline
roslib.load_manifest('raven_2_vision')
import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import *
from numpy import *
from math import *
from collections import namedtuple

import sys

class _Getch:
	"""Gets a single character from standard input.  Does not echo to the screen."""
	def __init__(self):
		try:
			self.impl = _GetchWindows()
		except ImportError:
			self.impl = _GetchUnix()

	def __call__(self): return self.impl()


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


class _GetchWindows:
	def __init__(self):
		import msvcrt

	def __call__(self):
		import msvcrt
		return msvcrt.getch()


def tb_to_mat(yaw, pitch, roll):
	Ryaw = matrix(
			[[cos(yaw), -sin(yaw), 0],
			[sin(yaw),  cos(yaw), 0],
			[0,		 0,		1]])
	Rpitch = matrix(
			 [[cos(pitch), 0, sin(pitch)],
			 [0,		  1, 0],
			[-sin(pitch), 0, cos(pitch)]])
	Rroll = matrix(
			[[1,  0,		  0],
			[0,  cos(roll), -sin(roll)],
			[0,  sin(roll),  cos(roll)]])
	tbmat = Ryaw * Rpitch * Rroll
	return vstack((hstack((tbmat,matrix([0,0,0]).transpose())),matrix([0,0,0,1])))

FLOAT_CLOSE_ENOUGH = 0.0001
def get_tb_angles(R) :
	tb_angles = namedtuple('tb_angles', ['yaw','pitch','roll','yaw_deg','pitch_deg','roll_deg'])
	angles_yaw = 0;
	angles_pitch = 0;
	angles_roll = 0;
	
	skip = False
	if fabs(R[0,1]-R[1,0]) < FLOAT_CLOSE_ENOUGH and fabs(R[0,2]-R[2,0]) < FLOAT_CLOSE_ENOUGH and fabs(R[1,2]-R[2,1]) < FLOAT_CLOSE_ENOUGH:
		#matrix is symmetric
		if fabs(R[0,1]+R[1,0]) < FLOAT_CLOSE_ENOUGH and fabs(R[0,2]+R[2,0]) < FLOAT_CLOSE_ENOUGH and fabs(R[1,2]+R[2,1]) < FLOAT_CLOSE_ENOUGH:
			#diagonal
			if R[0,0] > 0:
				if R[1,1] > 0:
					skip = True
				else:
					angles.roll = pi;
			elif R[1,1] > 0:
				angles.yaw = pi;
				angles.pitch = pi;
			else:
				angles.yaw = pi;
			skip=True
	
	if not skip:
		vx = R[0:3,0:3] * matrix([1,0,0]).transpose();
		vy = R[0:3,0:3] * matrix([0,1,0]).transpose();

		yaw = atan2(vx[1,0],vx[0,0]);
		pitch = atan2(-vx[2,0], sqrt(vx[0,0]*vx[0,0] + vx[1,0]*vx[1,0]));

		Ryaw = matrix(
					 [[cos(yaw), -sin(yaw), 0],
					 [sin(yaw),  cos(yaw), 0],
					 [0,		 0,		1]]);
		Rpitch = matrix(
				 [[cos(pitch), 0, sin(pitch)],
				 [0,		  1, 0],
				[-sin(pitch), 0, cos(pitch)]]);
		vyp = Ryaw * Rpitch * matrix([0,1,0]).transpose();
		vzp = Ryaw * Rpitch * matrix([0,0,1]).transpose();

		if vzp.transpose() * vy >= 0:
			coeff = 1
		else:
			coeff = -1

		roll = coeff * acos(vyp.transpose() * vy);
	
		angles_yaw = yaw;
		angles_pitch = pitch;
		angles_roll = roll;


	angles_yaw_deg = angles_yaw * 180. / pi;
	angles_pitch_deg = angles_pitch * 180. / pi;
	angles_roll_deg = angles_roll * 180. / pi;

	return tb_angles(angles_yaw,angles_pitch,angles_roll,angles_yaw_deg,angles_pitch_deg,angles_roll_deg)
	

class ChessboardFrame:

	def __init__(self,listener):
		self.listener = listener
		self.pub = rospy.Publisher('camera_pose', PoseStamped)
		self.pub_cb = rospy.Publisher('chessboard_world_pose', PoseStamped)
		self.br = tf.TransformBroadcaster()
		self.transform = None
		self.transform_frame = None
		
		self.x_offset = 0.2
		self.y_offset = -0.23071
		self.z_offset= 0
		
		self.pos_increment = 0.005
		
		self.roll = 0
		self.pitch = 0
		self.yaw = pi/2
		
		self.angle_increment = 1 * pi / 180
		
		self.invert_tf = False

	def pose_callback(self,msg):
		#print 'got msg'
		frame_id = msg.header
		pose = msg.pose
		p = tft.translation_matrix([pose.position.x,pose.position.y,pose.position.z])
		rot = tft.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
		
		Tcam_to_cb = dot(p,rot)
		#print 'Tcam_to_cb',Tcam_to_cb
		
		#Tworld_to_cb = dot(tft.translation_matrix([self.x_offset,self.y_offset,self.z_offset]),tft.euler_matrix(0,0,pi/2))
		Tworld_to_cb = dot(tft.translation_matrix([self.x_offset,self.y_offset,self.z_offset]),array(tb_to_mat(self.yaw, self.pitch, self.roll)))
		#print 'Tworld_to_cb',Tworld_to_cb
		
		if self.invert_tf:
			Tworld_to_cb = tft.inverse_matrix(Tworld_to_cb)
		
		Tworld_to_cam = dot(Tworld_to_cb,tft.inverse_matrix(Tcam_to_cb))
		#print 'Tworld_to_cam',Tworld_to_cam
		
		Tworld_to_cam_p = Tworld_to_cam[0:3,3]
		Tworld_to_cam_q = tft.quaternion_from_matrix(Tworld_to_cam)
		
		pub_msg = PoseStamped()
		pub_msg.header.stamp = msg.header.stamp
		pub_msg.header.frame_id = '/world'
		pub_msg.pose.position = Point(*(Tworld_to_cam_p.tolist()))
		pub_msg.pose.orientation = Quaternion(*(Tworld_to_cam_q.tolist()))
		
		#print pub_msg
		self.pub.publish(pub_msg)
		
		pub_cb_msg = PoseStamped()
		pub_cb_msg.header.stamp = msg.header.stamp
		pub_cb_msg.header.frame_id = '/world'
		pub_cb_msg.pose.position = Point(*(Tworld_to_cb[0:3,3].tolist()))
		pub_cb_msg.pose.orientation = Quaternion(*(tft.quaternion_from_matrix(Tworld_to_cb).tolist()))
		
		self.pub_cb.publish(pub_cb_msg)
		
		self.transform = Tworld_to_cam
		self.transform_frame = msg.header.frame_id
		
		self.br.sendTransform(Tworld_to_cam_p,Tworld_to_cam_q,msg.header.stamp,self.transform_frame,'/world')
		
		
		#print Tworld_to_cam_p.tolist() + Tworld_to_cam_q.tolist()

	def pub_tf(self):
		if self.transform is not None:
			Tworld_to_cam_p = self.transform[0:3,3]
			Tworld_to_cam_q = tft.quaternion_from_matrix(self.transform)
			self.br.sendTransform(Tworld_to_cam_p,Tworld_to_cam_q,rospy.Time.now(),self.transform_frame,'/world')


def print_help():
	print 'x & y: WASD'
	print 'z:    up   Q \t down  Z'
	print 'yaw   left Y \t right U'
	print 'pitch down P \t up    0'
	print 'roll  left E \t right R'
	print 'view current params:  V'
	print 'view/change position increment: I'
	print 'view/change angle increment:    K'
	print 'set positions or angles: ='
	print 'Flip transform:          -'
	print '---------------------------------'
	
if __name__ == '__main__':
	rospy.init_node('chessboard_frame')
	
	listener = tf.TransformListener()
	
	cf = ChessboardFrame(listener)
	
	rospy.Subscriber('chessboard_pose',PoseStamped,cf.pose_callback)
	
	print 'ready'
	
	print_help()
	
	while not rospy.is_shutdown():
		#cf.pub_tf()
		getch = _Getch()
		#cmd = '_';
		cmd = getch().lower()
		if ord(cmd) == 3:
			rospy.signal_shutdown('user terminated')
		else:
			if cmd == 'h':
				print_help()
			if cmd == 'a':
				cf.y_offset += cf.pos_increment
		 	elif cmd == 'd':
		 		cf.y_offset -= cf.pos_increment
	 		elif cmd == 's':
	 			cf.x_offset -= cf.pos_increment
	 		elif cmd == 'w':
	 			cf.x_offset += cf.pos_increment
 			elif cmd == 'q':
 				cf.z_offset += cf.pos_increment
			elif cmd == 'z':
				cf.z_offset -= cf.pos_increment
			elif cmd == 'y':
				cf.yaw += cf.angle_increment
			elif cmd == 'u':
				cf.yaw -= cf.angle_increment
			elif cmd == 'p':
				cf.pitch -= cf.angle_increment
			elif cmd == '0':
				cf.pitch += cf.angle_increment
			elif cmd == 'e':
				cf.roll -= cf.angle_increment
			elif cmd == 'r':
				cf.roll += cf.angle_increment
			elif cmd == 'v':
				if cf.invert_tf:
					print "**INVERTED**"
				print "Position offset:         ({0}, {1}, {2})".format(cf.x_offset,cf.y_offset,cf.z_offset)
				print "Position increment:       {0}".format(cf.pos_increment)
				print "Angles (yaw,pitch,roll): ({0}, {1}, {2})".format(cf.yaw,cf.pitch,cf.roll)
				print "Angle    increment:       {0}".format(cf.angle_increment)
				print '---------------------------------'
			elif cmd == 'i':
				print 'Current pos increment: {0}'.format(cf.pos_increment)
				l = raw_input('Enter value or hit return: ');
				if not l:
					try:
						cf.pos_increment = float(l)
					except Exception, e:
						pass
				print 'Value saved: {0}'.format(cf.pos_increment)
			elif cmd == 'k':
				print 'Current angle increment: {0}'.format(cf.angle_increment)
				l = raw_input('Enter value or hit return: ');
				if l:
					try:
						cf.angle_increment = float(l)
					except Exception, e:
						pass
				print 'Value saved: {0}'.format(cf.angle_increment)
			elif cmd == '-':
				cf.invert_tf = not cf.invert_tf
				if cf.invert_tf:
					print 'tf is now normal'
				else:
					print 'tf is now inverted'
			elif cmd == '=':
				print 'Choose position (P) or angles (A)'
				cmd = getch().lower()
				if cmd == 'p':
					l = raw_input('Enter new offsets or enter field: ').lower()
					if l:
						if l[0] == 'x':
							cf.x_offset = float(l[1:])
						elif l[0] == 'y':
							cf.y_offset = float(l[1:])
						elif l[0] == 'z':
							cf.z_offset = float(l[1:])
						else:
							if l.find(',') != -1:
								nums = l.split(',')
							else:
								nums = l.split(' ')
							cf.x_offset = float(nums[0])
							cf.y_offset = float(nums[1])
							cf.z_offset = float(nums[2])
				elif cmd == 'a':
					l = raw_input('Enter new angles or enter field (* for rad): ').lower()
					if l:
						conv = pi/180
						if l[0] == 'q':
							l = l[1:].strip()
							if l.find(',') != -1:
								nums = l.split(',')
							else:
								nums = l.split(' ')
							qmat = tft.quaternion_matrix([float(nums[0]),float(nums[1]),float(nums[2]),float(nums[3])])
							angles = get_tb_angles(qmat)
							print "setting angles to ({0},{1},{2})".format(angles.yaw_deg,angles.pitch_deg,angles.roll_deg)
							cf.yaw = angles.yaw
							cf.pitch = angles.pitch
							cf.roll = angles.roll
						if l[0] == '*':
							conv = 1
							l = l[1:].strip()
						if l[0] == 'y':
							cf.yaw = float(l[1:]) * conv
						elif l[0] == 'p':
							cf.pitch = float(l[1:]) * conv
						elif l[0] == 'r':
							cf.roll = float(l[1:]) * conv
						else:
							if l.find(',') != -1:
								nums = l.split(',')
							else:
								nums = l.split(' ')
							cf.yaw = float(nums[0]) * conv
							cf.pitch = float(nums[1]) * conv
							cf.roll = float(nums[2]) * conv
		rospy.sleep(0.1)