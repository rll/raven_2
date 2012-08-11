#!/usr/bin/env python
import roslib
roslib.load_manifest('raven_2_vision')
import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import *
from numpy import *
from math import *

import sys

class ChessboardFrame:

	def __init__(self,listener):
		self.listener = listener
		self.pub = rospy.Publisher('camera_pose', PoseStamped)
		self.pub_cb = rospy.Publisher('chessboard_world_pose', PoseStamped)
		self.br = tf.TransformBroadcaster()
		self.transform = None
		self.transform_frame = None
		

	def pose_callback(self,msg):
		print 'got msg'
		frame_id = msg.header
		pose = msg.pose
		p = tft.translation_matrix([pose.position.x,pose.position.y,pose.position.z])
		rot = tft.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
		
		Tcam_to_cb = dot(p,rot)
		#print 'Tcam_to_cb',Tcam_to_cb
		
		Tworld_to_cb = dot(tft.translation_matrix([0.2,-0.23071,0]),tft.euler_matrix(0,0,-pi/2))
		#print 'Tworld_to_cb',Tworld_to_cb
		
		Tworld_to_cam = dot(Tworld_to_cb,tft.inverse_matrix(Tcam_to_cb))
		#print 'Tworld_to_cam',Tworld_to_cam
		
		Tworld_to_cam_p = Tworld_to_cam[0:3,3]
		Tworld_to_cam_q = tft.quaternion_from_matrix(Tworld_to_cam)
		
		pub_msg = PoseStamped()
		pub_msg.header.stamp = msg.header.stamp
		pub_msg.header.frame_id = '/world'
		pub_msg.pose.position = Point(*(Tworld_to_cam_p.tolist()))
		pub_msg.pose.orientation = Quaternion(*(Tworld_to_cam_q.tolist()))
		
		self.pub.publish(pub_msg)
		
		pub_cb_msg = PoseStamped()
		pub_cb_msg.header.stamp = msg.header.stamp
		pub_cb_msg.header.frame_id = '/world'
		pub_cb_msg.pose.position = Point(*(Tworld_to_cb[0:3,3].tolist()))
		pub_cb_msg.pose.orientation = Quaternion(*(tft.quaternion_from_matrix(Tworld_to_cb).tolist()))
		
		self.pub_cb.publish(pub_cb_msg)
		
		self.transform = Tworld_to_cam
		self.transform_frame = msg.header.frame_id
		
		
		print Tworld_to_cam_p.tolist() + Tworld_to_cam_q.tolist()

	def pub_tf(self):
		if self.transform is not None:
			Tworld_to_cam_p = self.transform[0:3,3]
			Tworld_to_cam_q = tft.quaternion_from_matrix(self.transform)
			self.br.sendTransform(Tworld_to_cam_p,Tworld_to_cam_q,rospy.Time.now(),self.transform_frame,'/world')

if __name__ == '__main__':
	rospy.init_node('chessboard_frame')
	
	listener = tf.TransformListener()
	
	cf = ChessboardFrame(listener)
	
	rospy.Subscriber('chessboard_pose',PoseStamped,cf.pose_callback)
	
	print 'ready'
	while not rospy.is_shutdown():
		cf.pub_tf()
		rospy.sleep(1)