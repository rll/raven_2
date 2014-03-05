"""
Contains general useful functions
for conversions, etc.
"""

# Import required Python code.
import roslib
roslib.load_manifest('raven_2_utils')
import rospy
from visualization_msgs.msg import Marker
import tf.transformations as tft
import numpy as np
import math

import tfx

from raven_2_msgs.msg import *

import IPython

def createMarker(pose, id_):
    marker = Marker()
    marker.id = id_
    marker.header.frame_id = pose.header.frame_id
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.002
    marker.scale.y = 0.002
    marker.scale.z = 0.002
    marker.color.a = 1.0
    marker.color.r = 255
    marker.color.g = (id_) * 127
    marker.pose = pose.pose
    return marker

def withinBounds(ps0, ps1, transBound, rotBound, transFrame=None, rotFrame=None):
    """
    Returns if ps0 and ps1 (PoseStamped) are within translation and rotation bounds of each other

    Note: rotBound is in degrees
    """

    dPose = tfx.pose(deltaPose(ps0, ps1, transFrame, rotFrame))
    
    deltaPositions = dPose.position.list
    for deltaPos in deltaPositions:
        if abs(deltaPos) > transBound:
            return False

    between = angleBetweenQuaternions(tfx.tb_angles([0,0,0,1]).msg, dPose.orientation)
    if between > rotBound:
        return False
    
    return True

def checkBounds(gripperPose, foamPose, transBound, rotBound, transFrame=None, rotFrame=None):
    delPose = deltaPose(gripperPose, foamPose, transFrame, rotFrame).position.list
    bounds = [0.015, 0.007, 0.015]
    for i in range(3):
        if (math.fabs(delPose[i])>bounds[i]):
            return False
    return True

def deltaPose(currPose, desPose, posFrame=None, rotFrame=None):
    """
    Returns pose0 - pose1
    """

    currPose, desPose = tfx.pose(currPose), tfx.pose(desPose)
    currPoseFrame, desPoseFrame = currPose.frame, desPose.frame
    
    currPos, desPos = currPose.position, desPose.position
    currRot, desRot = currPose.orientation, desPose.orientation

    if posFrame is not None and currPoseFrame is not None:
        tf_currPos_to_posFrame = tfx.lookupTransform(posFrame, currPoseFrame, wait=10)
        currPos = tf_currPos_to_posFrame * currPos

        tf_desPos_to_posFrame = tfx.lookupTransform(posFrame, desPoseFrame, wait=10)
        desPos = tf_desPos_to_posFrame * desPos

    if rotFrame is not None and currPoseFrame is not None:
        tf_currRot_to_rotFrame = tfx.lookupTransform(rotFrame, currPoseFrame, wait=10)
        currRot = tf_currRot_to_rotFrame * currRot

        tf_desRot_to_rotFrame = tfx.lookupTransform(rotFrame, desPoseFrame, wait=10)
        desRot = tf_desRot_to_rotFrame * desRot

    deltaPosition = desPos.array - currPos.array
    
    currQuat, desQuat = tfx.tb_angles(currRot).quaternion, tfx.tb_angles(desRot).quaternion
    deltaQuat = tft.quaternion_multiply(tft.quaternion_inverse(currQuat), desQuat)

    deltaPose = tfx.pose(deltaPosition, deltaQuat)

    return deltaPose
    
def endPose(currPose, deltaPose, frame=None):
    
    currPose = tfx.pose(currPose)
    deltaPose = tfx.pose(deltaPose)

    if frame:
        currPose = convertToFrame(currPose, frame)
        deltaPose = convertToFrame(deltaPose, frame)

    if currPose.frame and deltaPose.frame and currPose.frame != deltaPose.frame:
        deltaPose = convertToFrame(deltaPose, currPose.frame)

    endPosition = currPose.position + deltaPose.position
    endQuatMat = currPose.orientation.matrix * deltaPose.orientation.matrix

    endPose = tfx.pose(endPosition, endQuatMat)

    return endPose
    
    
def angleBetweenQuaternions(quat0, quat1):
    """
    Returns angle between quat0 and quat1 in degrees
    """
    q0 = np.array([quat0.x, quat0.y, quat0.z, quat0.w])
    q1 = np.array([quat1.x, quat1.y, quat1.z, quat1.w])

    try:
        theta = math.acos(2*np.dot(q0,q1)**2 - 1)
    except ValueError:
        return 0

    theta = theta*(180.0/math.pi)

    return theta

def convertToFrame(p, frame):
    """
    Takes in a tfx pose stamped and returns it in frame
    """
    p = tfx.pose(p)
    
    if p.frame and p.frame != frame:
        try:
            tf_pframe_to_frame = tfx.lookupTransform(frame, p.frame, wait=10)
        except Exception, e:
            print frame, p.frame
            raise e
        p = tf_pframe_to_frame * p

    return p

def openraveTransformFromTo(robot, poseMatInRef, refLinkName, targLinkName):
    # ref -> world
    refFromWorld = robot.GetLink(refLinkName).GetTransform()

    # target -> world
    targFromWorld = robot.GetLink(targLinkName).GetTransform()

    # target -> ref
    targFromRef = np.dot(np.linalg.inv(targFromWorld), refFromWorld)

    poseMatInTarg = np.dot(targFromRef, poseMatInRef)
    return np.array(poseMatInTarg)
    
def setWithinLimits(val, lower, upper, increment):
    while val >= upper:
        val -= increment
        
    while val <= lower:
        val += increment
        
    return val

class Timeout():
    def __init__(self, timeoutTime):
        """
        timeoutTime is integer of how long until times out
        """
        self.timeoutTime = timeoutTime

    def start(self):
        """
        Restarts timeout every time this method is called
        """
        self.endTime = rospy.Time.now() + rospy.Duration(self.timeoutTime)

    def hasTimedOut(self):
        """
        returns true if time since start method called is
        greater than the current time
        """
        return rospy.Time.now() > self.endTime 


def plot_transform(env, T, s=0.1):
    """
    Plots transform T in openrave environment.
    S is the length of the axis markers.
    """
    h = []
    x = T[0:3,0]
    y = T[0:3,1]
    z = T[0:3,2]
    o = T[0:3,3]
    h.append(env.drawlinestrip(points=np.array([o, o+s*x]), linewidth=3.0, colors=np.array([(1,0,0),(1,0,0)])))
    h.append(env.drawlinestrip(points=np.array([o, o+s*y]), linewidth=3.0, colors=np.array(((0,1,0),(0,1,0)))))
    h.append(env.drawlinestrip(points=np.array([o, o+s*z]), linewidth=3.0, colors=np.array(((0,0,1),(0,0,1)))))
    return h



def testAngleBetween():
    quat0 = tfx.tb_angles(-82,90,98).msg
    quat1 = tfx.tb_angles(-3,90,-8).msg

    theta = angleBetweenQuaternions(quat0, quat1)

    print("theta = {0}".format(theta))

if __name__ == '__main__':
    testAngleBetween()
