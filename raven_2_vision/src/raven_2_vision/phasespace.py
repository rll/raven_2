#!/usr/bin/env python

# Import required Python code.
import roslib
import argparse
roslib.load_manifest('raven_2_vision')
import rospy

from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
import tfx
import time
import threading

from OWL import *
from math import *
import sys

import IPython

MARKER_COUNT = 30
SERVER_NAME = "10.116.173.110"
INIT_FLAGS = 0
STREAMING_FREQUENCY = 100

GRIPPER_MARKER_IDS = [4, 11, 10]
REGISTRATION_MARKER_IDS = [9, 7, 6]

# pose of the Raven screw relative to 0 link as of 02/13/14
SCREW_X_0LINK = 0.3007
SCREW_Y_0LINK= 0.0754
SCREW_Z_0LINK = 0.02000

SCREW_ROLL_0LINK = -90
SCREW_PITCH_0LINK = 0
SCREW_YAW_0LINK = 0

SCREW_TRANSLATION = np.array([SCREW_X_0LINK, SCREW_Y_0LINK, SCREW_Z_0LINK])
SCREW_ROTATION = np.array([SCREW_ROLL_0LINK, SCREW_PITCH_0LINK, SCREW_YAW_0LINK])

SCREW_FRAME = 'screw'
PHASESPACE_FRAME = 'phasespace'
PHASESPACE_GRIPPER_FRAME = 'phasespace_gripper'

'''
Created on Feb 13, 2014

@author: jmahler
'''

class PhasespaceTracker(object):
    def __init__(self, verbose=False, arm='L', register=False):
        # setup variables
        self.verbose = verbose
        self.arm = arm
        self.register = register
        self.thread = None
        self.messageTime = 0
        
        # setup pose publishers
        self.gripper_pose_pub = rospy.Publisher('phasespace_gripper_pose_' + self.arm, PoseStamped)
        self.camera_pose_pub = rospy.Publisher('phasespace_camera_pose_' + self.arm, PoseStamped)
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        
        # connect to server
        print("owlInit: ", owlInit(SERVER_NAME, INIT_FLAGS))
        
        # create a tracker
        tracker = 0
        owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER)

        for i in range(72): 
            # add the points we want to track to the tracker
            owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i)
            pass
        
        # activate tracker
        owlTracker(tracker, OWL_ENABLE)
         
        # set frequency
        owlSetFloatv(OWL_FREQUENCY, [STREAMING_FREQUENCY])
        
        # start streaming
        owlSetIntegerv(OWL_STREAMING, [OWL_ENABLE])
         
        # turn on timestamps
        owlSetIntegerv(OWL_TIMESTAMP, [OWL_ENABLE])
        
        # check for errors
        if owlGetError() != OWL_NO_ERROR:
            rospy.loginfo('Phasespace initialization failed...')
            sys.exit(0)
            pass
        
        # start other thread
        self.start()
        
    def start(self):
        self.running = True
        self.finished = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        
    def stop(self):
        self.running = False
    #    while not self.finished:
       #     time.sleep(1)
        
    def run(self):
        while(self.running):
            if True:
                self.broadcaster.sendTransform(SCREW_TRANSLATION, tfx.tb_to_quat(SCREW_YAW_0LINK, SCREW_PITCH_0LINK, SCREW_ROLL_0LINK), rospy.Time.now(), SCREW_FRAME, '0_link')
                
                # get markers and accompanying metadata
                frame = owlGetIntegerv(OWL_FRAME_NUMBER)[0]
                timestamp = owlGetIntegerv(OWL_TIMESTAMP)
                if len(timestamp) > 0:
                    timeInSeconds = timestamp[1]
                    timeInSeconds += timestamp[2] * 1e-6
                    self.messageTime = rospy.Time.from_sec(timeInSeconds)
                
                markers = owlGetMarkers()
                
                if owlGetError() != OWL_NO_ERROR:
                    rospy.loginfo('Getting markers failed...')
                    sys.exit(0)
                    pass
                
                gripperMarkers = []
                registrationMarkers = []
                
                # find the correct markers
                for i in range(markers.size()):
                    m = markers[i]
                    
                    if m.cond > 0:
                        if m.id in GRIPPER_MARKER_IDS:
                            gripperMarkers.append(m)
                        elif m.id in REGISTRATION_MARKER_IDS:
                            registrationMarkers.append(m)
    
                        if self.verbose:
                            print("marker %06d %d: %f %f %f %f" % (frame, m.id, m.cond, m.x, m.y, m.z))
                            
                # get the gripper pose (in phasespace basis)
                if len(gripperMarkers) == 3:
                    gTrans, gRot = self.poseFromMarkers(gripperMarkers, GRIPPER_MARKER_IDS, rotate=True)
                    
                    gripperPosePhasespace = tfx.pose(gTrans, gRot, frame=PHASESPACE_FRAME, stamp=self.messageTime)
                    self.gripper_pose_pub.publish(gripperPosePhasespace.msg.PoseStamped())
                    
                    self.broadcaster.sendTransform(gTrans, gRot, self.messageTime, PHASESPACE_GRIPPER_FRAME, PHASESPACE_FRAME)
                
                # get the registration marker pose (in phasespace basis)
                if self.register and len(registrationMarkers) == 3:
                    rTrans, rRot = self.poseFromMarkers(registrationMarkers, REGISTRATION_MARKER_IDS)
                    screwPosePhasespaceFrame = tfx.pose(rTrans, rRot)
                    phasespacePoseScrewFrame = screwPosePhasespaceFrame.inverse()
                    #IPython.embed()
                    self.broadcaster.sendTransform(phasespacePoseScrewFrame.translation, phasespacePoseScrewFrame.rotation, self.messageTime, PHASESPACE_FRAME, SCREW_FRAME)
         #   except:
        #        rospy.loginfo('Exception thrown while getting markers...')
                
            self.finished = True

    def poseFromMarkers(self, markers, target_ids, rotate=False):
        if len(markers) >= 3:
            points = [np.zeros((3,1)), np.zeros((3,1)), np.zeros((3,1))]
            
            for m in markers:
                points[target_ids.index(m.id)] = np.array([m.x, m.y, m.z])
            
            leftPoint = points[0]
            centerPoint = points[1]
            rightPoint = points[2]
            
            leftVec = leftPoint - centerPoint
            rightVec = rightPoint - centerPoint
            leftVec = leftVec / np.linalg.norm(leftVec) 
            rightVec = rightVec / np.linalg.norm(rightVec) 
            
            zAxis = np.cross(leftVec, rightVec)    
            yAxis = (leftVec + rightVec) / 2
            yAxis= yAxis / np.linalg.norm(yAxis)
            xAxis = np.cross(yAxis, zAxis)
            
            rotMat = np.vstack((xAxis, yAxis, zAxis))
            tbRot = tfx.tb_angles(rotMat).matrix
            if rotate:
                tbRot = self.rotate(180, "roll", tbRot)
                tbRot = self.rotate(-90, "yaw", tbRot)
            quat = tfx.tb_angles(tbRot).quaternion
            
            return 1e-3*centerPoint, quat
        return None
        
    def rotate(self, angle, axis, matrix):
        """ 
        Takes a rotation matrix and rotates it a certain angle about a certain axis 
        """
        if axis == "yaw":
            rot = tfx.tb_angles(angle, 0, 0).matrix
        elif axis == "pitch":
            rot = tfx.tb_angles(0, angle, 0).matrix
        elif axis == "roll":
            rot = tfx.tb_angles(0, 0, angle).matrix
        return matrix*rot
        

if __name__ == '__main__':
    rospy.init_node('phasespace_tracker')
    
    p = PhasespaceTracker(register=False)
    rospy.spin()
    p.stop()
    
    #IPython.embed()