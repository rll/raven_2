#!/usr/bin/env python


import roslib
roslib.load_manifest('raven_2_trajectory')
import rospy
from math import *
import os

from raven_2_msgs.msg import * 
from geometry_msgs.msg import *
from std_msgs.msg import Header

from visualization_msgs.msg import Marker

from raven_2_control import kinematics as kin
from raven_2_trajectory.msg import TrajoptCall

import openravepy as rave
import trajoptpy
import cloudprocpy
import json
import numpy as np
import copy

import trajoptpy.make_kinbodies as mk
from trajoptpy.check_traj import traj_is_safe

import threading

import ctypes
import math
import struct

import tf
import tfx

from raven_2_msgs.msg import Constants
from raven_2_utils import raven_util
from raven_2_utils import raven_constants
import raven_2_calibration

from sensor_msgs.msg import PointCloud2, PointField

import IPython

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def transformRelativePoseForIk(manip, poseMatrix, refLinkName, targLinkName):
    """
    Adapted from PR2.py

    Returns transformed poseMatrix that can be used in openrave IK
    """
    robot = manip.GetRobot()
    
    # world <- ref
    refLink = robot.GetLink(refLinkName)
    worldFromRefLink = refLink.GetTransform()

    # world <- targ
    targLink = robot.GetLink(targLinkName)
    worldFromTargLink = targLink.GetTransform()

    # targ <- EE
    worldFromEE = manip.GetEndEffectorTransform()
    targLinkFromEE = np.dot(np.linalg.inv(worldFromTargLink), worldFromEE)
    
    # world <- EE
    refLinkFromTargLink = poseMatrix
    newWorldFromEE = np.dot(np.dot(worldFromRefLink, refLinkFromTargLink), targLinkFromEE)

    return newWorldFromEE

def jointRequest(n_steps, endJointPositions, startPoses, endPoses, toolFrames, manips, approachDirs=None, approachDist=0.02):
    """
    approachDirs is a list of 3d vectors dictating approach direction
    w.r.t. 0_link
    To come from above, approachDir = np.array([0,0,1])
    """
    n_steps = n_steps if n_steps > 5 else 5
    approachDirs = approachDirs or [None for _ in range(len(endPoses))]
    
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "active",
            "start_fixed" : True
            },
        "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]}
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [100],
                        "continuous" : True,
                        "dist_pen" : [0.001]
                        }
                    },
               {
                "type" : "collision",
                "params" : {
                    "coeffs" : [100],
                    "continuous" : False,
                    "dist_pen" : [0.001]
                    }
                },
                ],
        "constraints" : [
            {
                "type" : "joint", # joint-space target
                "params" : {"vals" : endJointPositions } # length of vals = # dofs of manip
                }
                
            ],
        "init_info" : {
            "type" : "straight_line",
            "endpoint" : endJointPositions
            }
        }
    
    for startPose, endPose, toolFrame, manip, approachDir in zip(startPoses, endPoses, toolFrames, manips, approachDirs):
            if approachDir is not None and tfx.point(np.array(endPose.position.list) - np.array(startPose.position.list)).norm > approachDist:
                #pose = tfx.pose(endPose)
                endPose = tfx.pose(endPose)
                approachPosition = endPose.position + approachDir/np.linalg.norm(approachDir)*approachDist
                pose = tfx.pose(approachPosition, endPose.orientation, frame=endPose.frame, stamp=endPose.stamp)
                
                print('endPosition: {0}'.format(endPose.position.list))
                print('approachPosition: {0}'.format(approachPosition.list))
            
                convPose = tfx.pose(transformRelativePoseForIk(manip, pose.matrix, pose.frame, toolFrame),frame=toolFrame)
            
                desPos = convPose.position.list
                desQuat = convPose.orientation
                wxyzQuat = [desQuat.w, desQuat.x, desQuat.y, desQuat.z]
                
                frame = toolFrame
                timestep = int(.75*n_steps) if int(.75*n_steps) < n_steps-2 else n_steps-3
                
                # cost or constraint?
                request["constraints"].append({
                            "type": "pose",
                            "name" : "target_pose",
                            "params" : {"xyz" : desPos,
                                        "wxyz" : wxyzQuat,
                                        "link" : frame,
                                        "rot_coeffs" : [0,0,0],
                                        "pos_coeffs" : [100000,100000,100000],
                                        "timestep" : timestep
                                        }
                            })

    return request





class RavenPlanner:
    rosJointTypes = [Constants.JOINT_TYPE_SHOULDER,
                     Constants.JOINT_TYPE_ELBOW,
                     Constants.JOINT_TYPE_INSERTION,
                     Constants.JOINT_TYPE_ROTATION,
                     Constants.JOINT_TYPE_PITCH,
                     Constants.JOINT_TYPE_YAW]

    raveJointNamesPrefixes = ["shoulder",
                              "elbow",
                              "insertion",
                              "tool_roll",
                              "wrist_joint",
                              "grasper_yaw"]
    
    defaultJointPositions = [.512, 1.6, -.2, .116, .088, 0]
    defaultJoints = dict([(jointType,jointPos) for jointType, jointPos in zip(rosJointTypes,defaultJointPositions)])

    def __init__(self, armNames, errorModel=None, thread=True, withWorkspace=False, addNoise=False):
        if isinstance(armNames,basestring):
            armNames = [armNames]
        self.armNames = sorted(armNames)
        self.errorModel = errorModel
        self.refFrame = raven_constants.Frames.Link0
        self.addNoise = addNoise

        self.env = rave.Environment()
        #self.env.SetViewer('qtcoin')

        rospy.loginfo('Before loading model')
        if withWorkspace:
            ravenFile = os.path.join(roslib.packages.get_pkg_subdir('RavenDebridement','models'),'raven_with_workspace.zae')
        else:
            ravenFile = os.path.join(roslib.packages.get_pkg_subdir('RavenDebridement','models'),'myRaven.xml')
        #ravenFile = '/home/gkahn/ros_workspace/RavenDebridement/models/myRaven.xml'
        self.env.Load(ravenFile)
        rospy.loginfo('After loading model')
        
        self.robot = self.env.GetRobots()[0]
        
        self.invKinArm = dict()
        self.toolFrame = dict()
        self.manipName = dict()
        self.manip = dict()
        self.manipJoints = dict()
        self.raveJointNames = dict()
        self.raveJointTypes = dict()
        self.raveJointTypesToRos = dict()
        self.rosJointTypesToRave = dict()
        self.raveGrasperJointNames = dict()
        self.raveGrasperJointTypes = dict()
        
        self.trajRequest = dict()
        
        self.trajEndJoints = dict()
        self.trajEndGrasp = dict()
        self.trajEndPose = dict()
        
        self.trajStartJoints = dict()
        self.trajStartGrasp = dict()
        self.trajStartPose = dict()
        
        self.trajSteps = dict()
        
        self.jointTraj = dict() # for debugging
        self.poseTraj = dict()
        self.deltaPoseTraj = dict()
        
        self.approachDir = dict()
        
        activeDOFs = []
        for armName in self.armNames:
            self._init_arm(armName)
            activeDOFs += self.raveJointTypes[armName]
            
        self.robot.SetActiveDOFs(activeDOFs)
        
        self.currentState = None
        rospy.Subscriber(raven_constants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)
        
        self.start_pose_pubs = dict((armName, rospy.Publisher('planner_%s_start' % armName,PoseStamped)) for armName in self.armNames)
        self.end_pose_pubs = dict((armName, rospy.Publisher('planner_%s_end' % armName,PoseStamped)) for armName in self.armNames)
        
        self.trajopt_pub = rospy.Publisher('trajopt',TrajoptCall)
        self.trajopt_marker_pub = rospy.Publisher('trajopt_marker',Marker)
        
        if withWorkspace:
            # setup variables for kinect constraint generation
            self.cloud_id = 0
            self.num_cd_components = 150
            self.decimation_rate = 0.15
            self.geom_type = 'cd'
            self.kinect_topic = '/camera/depth/points'
            self.kinect_depth_frame = '/camera_depth_optical_frame'
            self.robot_frame = '/world'
            self.cd_body_names = []
        
            # get transform to apply to cloud - it is in the kinect frame by default
            self.T_kinect_to_robot = tfx.lookupTransform(self.robot_frame, self.kinect_depth_frame, wait=20)
            
            workspace_file = rospy.get_param('workspace_constraints')
            workspace_file = os.path.join(roslib.packages.get_pkg_subdir('raven_2_params', 'data'), workspace_file)
            self.load_workspace(workspace_file)
                
        self.lock = threading.RLock()
        if thread:
            self.thread = threading.Thread(target=self.optimizeLoop)
            self.thread.setDaemon(True)
            self.thread.start()

        #rospy.Subscriber(self.kinect_topic, PointCloud2, self.kinect_callback)

    def _init_arm(self, armName):
        if armName == raven_constants.Arm.Left:
            self.invKinArm[armName] = Constants.ARM_TYPE_GOLD
            self.toolFrame[armName] = raven_constants.OpenraveLinks.LeftTool
            self.manipName[armName] = 'left_arm'
        else:
            self.invKinArm[armName] = Constants.ARM_TYPE_GREEN
            self.toolFrame[armName] = raven_constants.OpenraveLinks.RightTool
            self.manipName[armName] = 'right_arm'
        self.robot.SetActiveManipulator(self.manipName[armName])
        self.manip[armName] = self.robot.GetActiveManipulator()
        self.manipJoints[armName] = self.robot.GetJoints(self.manip[armName].GetArmJoints())


        self.raveJointNames[armName] = ['{0}_{1}'.format(name, armName[0].upper()) for name in self.raveJointNamesPrefixes]

        self.raveJointTypes[armName] = [self.robot.GetJointIndex(name) for name in self.raveJointNames[armName]]
        self.raveJointTypesToRos[armName] = dict((rave,ros) for rave,ros in zip(self.raveJointTypes[armName], self.rosJointTypes))
        self.rosJointTypesToRave[armName] = dict((ros,rave) for ros,rave in zip(self.rosJointTypes, self.raveJointTypes[armName]))
        
        self.raveGrasperJointNames[armName] = ['grasper_joint_1_{0}'.format(armName[0].upper()), 'grasper_joint_2_{0}'.format(armName[0].upper())]
        self.raveGrasperJointTypes[armName] = [self.robot.GetJointIndex(name) for name in self.raveGrasperJointNames[armName]]
        
        self.approachDir[armName] = None
        
        self.updateOpenraveJoints(armName, dict(self.defaultJoints), grasp=0)

        self.trajRequest[armName] = False

    def _ravenStateCallback(self, msg):
        self.currentState = msg
    
    def getCurrentPose(self, armName=None):
        if not self.currentState:
            return None
        currentPose = {}
        for arm in self.currentState.arms:
            armPose = tfx.pose(arm.tool.pose,header=self.currentState.header)
            if armName is None:
                currentPose[arm.name] = armPose
            elif arm.name == armName:
                currentPose = armPose
                break
        return currentPose
                    
    def getCurrentGrasp(self, armName = None):
        if not self.currentState:
            return None
        currentGrasp = {}
        for arm in self.currentState.arms:
            armGrasp = dict((joint.type, joint.position) for joint in arm.joints)[Constants.JOINT_TYPE_GRASP]
            if armName is None:
                currentGrasp[arm.name] = armGrasp
            elif arm.name == armName:
                currentGrasp = armGrasp
                break
        print currentGrasp
        return currentGrasp
                    
    def getCurrentJoints(self, armName=None):
        if not self.currentState:
            return None
        currentJoints = {}
        for arm in self.currentState.arms:
            armJoints = dict((joint.type, joint.position) for joint in arm.joints)
            if armName is None:
                currentJoints[arm.name] = armJoints
            elif arm.name == armName:
                currentJoints = armJoints
                break
        return currentJoints
    
    def waitForState(self):
        if not self.currentState:
            print 'waiting for ravenstate'
            while self.currentState is None and not rospy.is_shutdown():
                rospy.sleep(.05)
            print 'got it!'
            

    
    def getJointsFromPose(self, armName, pose, grasp, quiet=False):
        """
        Calls IK server and returns a dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Needs to return finger1 and finger2
        """

        pose = raven_util.convertToFrame(tfx.pose(pose), self.refFrame)
        
        joints = kin.invArmKin(armName, pose, grasp)
        
        if joints is None:
            rospy.loginfo('IK failed!')
            if quiet:
                return None
            else:
                raise RuntimeError("IK failed!")
        
        return joints
        

    def updateOpenraveJoints(self, armName, joints1=None, grasp=None):
        """
        Updates the openrave raven model using rosJoints

        rosJoints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Updates based on: shoulder, elbow, insertion,
                          rotation, pitch, yaw, grasp (self.currentGrasp)
        """
        rosJoints = joints1
        
        if rosJoints is None:
            rosJoints = self.getCurrentJoints(armName)
            if rosJoints is None:
                return
        if grasp is None:
            grasp = pi/2.
            #grasp = self.getCurrentGrasp(armName)
        
        raveJointTypes = []
        jointPositions = []
        for rosJointType, jointPos in rosJoints.items():
            if self.rosJointTypesToRave[armName].has_key(rosJointType):
                raveJointType = self.rosJointTypesToRave[armName][rosJointType]
                raveJointTypes.append(raveJointType)
                
                # since not a hard limit, must do this
                if rosJointType == Constants.JOINT_TYPE_ROTATION:
                    lim = self.robot.GetJointFromDOFIndex(raveJointType).GetLimits()
                    limLower, limUpper = lim[0][0], lim[1][0]
                    jointPos = raven_util.setWithinLimits(jointPos, limLower, limUpper, 2*pi)
                
                jointPositions.append(jointPos)
                

        # for opening the gripper
        raveJointTypes += self.raveGrasperJointTypes[armName]
        
        if armName == raven_constants.Arm.Left:
            jointPositions += [grasp/2, grasp/2]
        else:
            jointPositions += [grasp/2, -grasp/2]

        rospy.loginfo('Setting joints')
        self.robot.SetJointValues(jointPositions, raveJointTypes)
        rospy.loginfo('Done setting joints')

    def jointTrajToDicts(self, armName, jointTrajArray, **kwargs):
        """
        Converts a numpy array trajectory
        to a list of joint dictionaries

        dicts contain ros joints1:
        shoulder, elbow, insertion,
        rotation, pitch, finger1, finger2
        """
        startGrasp = kwargs.get('startGrasp',kwargs.get('grasp'))
        endGrasp = kwargs.get('startGrasp',kwargs.get('grasp'))
        if startGrasp is None:
            startGrasp = self.getCurrentGrasp(armName)
        if endGrasp is None:
            endGrasp = startGrasp
        
        grasps = np.linspace(startGrasp, endGrasp, len(jointTrajArray))
        jointTrajDicts = []
        for trajIndex in range(len(jointTrajArray)):
            grasp = grasps[trajIndex]
            
            waypointJointPositions = list(jointTrajArray[trajIndex])
            
            # since yaw is pseudo-joint, convert to finger1 and finger2
            yaw = waypointJointPositions[-1]
            finger1 = yaw - grasp/2
            finger2 = -(yaw + grasp/2)
            if armName == raven_constants.Arm.Left:
                finger1 = -finger1
                finger2 = -finger2
            # try keeping yaw
            waypointJointPositions = waypointJointPositions[:] + [finger1, finger2]
            rosJointTypesPlusFingers = self.rosJointTypes[:] + [Constants.JOINT_TYPE_GRASP_FINGER1, Constants.JOINT_TYPE_GRASP_FINGER2]
            
            waypointDict = dict(zip(rosJointTypesPlusFingers, waypointJointPositions))
            jointTrajDicts.append(waypointDict)

        return jointTrajDicts
    
    def jointDictsToPoses(self, armName, jointTrajDicts):
        """
        Converts a list of joint trajectory dicts
        to a list of poses using openrave FK
        """
        poseList = []
        for jointDict in jointTrajDicts:
            pose, grasp = kin.fwdArmKin(armName, jointDict)
            poseList.append(pose)
        
        return poseList
    
    def posesToDeltaPoses(self, poseList):
        """
        p0 -> p1 -> p2 -> ....
        to
        delta(p0->p1) -> delta(p0->p2) -> delta(p0->p3)...
        """
        startPose = poseList[0]
        return [raven_util.deltaPose(startPose, pose) for pose in poseList[1:]]
            
    
    def optimize1(self, n_steps, trajStartJoints, trajEndJoints):
        msg = TrajoptCall()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/0_link'
        
        startJointPositions = []
        endJointPositions = []
        startPoses = []
        endPoses = []
        toolFrames = []
        manips = []
        approachDirs = []
        for armName in self.armNames:
            endJoints = trajEndJoints[armName]
            if endJoints is None:
                print trajEndJoints, self.trajRequest
            endJoints = dict([(jointType,jointPos) for jointType, jointPos in endJoints.items() if jointType in self.rosJointTypes])
            
            startJoints = trajStartJoints[armName]
            startJoints = dict([(jointType,jointPos) for jointType, jointPos in startJoints.items() if jointType in self.rosJointTypes])
            
            print 'start joints %s: %s' % (armName, [startJoints[k] for k in sorted(startJoints.keys())])
            print 'end joints %s: %s' % (armName, [endJoints[k] for k in sorted(endJoints.keys())])
            
            for raveJointType in self.manip[armName].GetArmIndices():
                rosJointType = self.raveJointTypesToRos[armName][raveJointType]
                endJointPositions.append(endJoints[rosJointType])
                startJointPositions.append(startJoints[rosJointType])
            
            startPoses.append(self.trajStartPose[armName])
            endPoses.append(self.trajEndPose[armName])
            toolFrames.append(self.toolFrame[armName])
            manips.append(self.manip[armName])
            approachDirs.append(self.approachDir[armName])
            
            self.updateOpenraveJoints(armName, trajStartJoints[armName])
            
            if armName == 'L':
                msg.start_L = self.trajStartPose[armName]
                msg.end_L = self.trajEndPose[armName]
            else:
                msg.start_R = self.trajStartPose[armName]
                msg.end_R = self.trajEndPose[armName]
                
        #request = jointRequest(n_steps, endJointPositions)
        request = jointRequest(n_steps, endJointPositions, startPoses, endPoses, toolFrames, manips, approachDirs=approachDirs, approachDist=0.03)
    
        #IPython.embed()
    
        # convert dictionary into json-formatted string
        s = json.dumps(request)
        #print s
        #print self.robot.GetActiveDOFValues()
        #return
        # create object that stores optimization problem
        rospy.loginfo('Constructing Problem')
        prob = trajoptpy.ConstructProblem(s, self.env)
        # do optimization
        rospy.loginfo('Optimizing Problem')
        result = trajoptpy.OptimizeProblem(prob)
    
        # check trajectory safety
        from trajoptpy.check_traj import traj_is_safe
        prob.SetRobotActiveDOFs()
        if not traj_is_safe(result.GetTraj(), self.robot):
            rospy.loginfo('Trajopt trajectory is not safe. Trajopt failed!')
            for armName in self.armNames:
                self.poseTraj[armName] = None
                self.deltaPoseTraj[armName] = None
        else:
            startIndex = 0
            for armName in self.armNames:
                endIndex = startIndex + len(self.manipJoints[armName])
                
                graspKwargs = {}
                if self.trajStartGrasp[armName] is not None:
                    graspKwargs['startGrasp'] = self.trajStartGrasp[armName]
                if self.trajEndGrasp[armName] is not None:
                    graspKwargs['endGrasp'] = self.trajEndGrasp[armName]
                
                armJointTrajArray = result.GetTraj()[:,startIndex:endIndex]
                jointTrajDicts = self.jointTrajToDicts(armName, armJointTrajArray, **graspKwargs)
                self.jointTraj[armName] = jointTrajDicts # for debugging
                poseTraj = self.jointDictsToPoses(armName, jointTrajDicts)
                #if self.addNoise:
                #    poseTraj = self.perturbTrajectory(armName, poseTraj)
                correctedPoseTraj = copy.copy(poseTraj) #self.correctPoseTrajectory(armName, poseTraj) # apply calibrated error model to trajectory
                self.poseTraj[armName] = copy.copy(correctedPoseTraj)
                deltaPoseTraj = self.posesToDeltaPoses(correctedPoseTraj)
                self.deltaPoseTraj[armName] = deltaPoseTraj
                
                startIndex = endIndex
                
                if armName == 'L':
                    msg.traj_L = [p.msg.Pose() for p in self.poseTraj[armName]]
                else:
                    msg.traj_R = [p.msg.Pose() for p in self.poseTraj[armName]]
        
        self.trajopt_pub.publish(msg)
        marker = Marker()
        marker.header.frame_id = '/0_link'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.ns = 'trajopt'
        marker.id = 0
        marker.scale.x = 0.002
        marker.scale.y = 0.002
        marker.color.a = 0.75
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        for arm in self.poseTraj:
            for p in self.poseTraj[arm]:
                marker.points.append(p.position.msg.Point())
        #marker.lifetime = rospy.Duration(1.5)
        self.trajopt_marker_pub.publish(marker)
        print 'FINISHED OPTIMIZATION'
    
    def optimize2(self, n_steps):
        return self.optimize1(n_steps, self.trajStartJoints, self.trajEndJoints)
    
    def optimize3(self):
        n_steps = max(self.trajSteps.values())
        return self.optimize2(n_steps)
    
    def optimizeLoop(self, once=False):
        while not rospy.is_shutdown():
            rospy.sleep(.05)
            
            # block until all traj submissions received
            while False in [self.trajRequest.get(armName,False) for armName in self.armNames]:
                if rospy.is_shutdown():
                    return
                rospy.sleep(0.05)
            
            print "it's go time"
            self.optimize3()
            
            with self.lock:
                for armName in self.armNames:
                    self.trajRequest[armName] = False
            
            if once:
                break
    
    def setStartAndEndJoints(self, armName, startJoints, endJoints, **kwargs):
        self.trajStartGrasp[armName] = kwargs.get('startGrasp',kwargs.get('grasp',None))
        self.trajEndGrasp[armName] = kwargs.get('endGrasp',kwargs.get('grasp',None))
        self.trajStartJoints[armName] = startJoints
        self.trajEndJoints[armName] = endJoints
    
    def setStartJointsAndEndPose(self, armName, startJoints, endPose,**kwargs):
        startGrasp = kwargs.get('startGrasp',kwargs.get('grasp',0))
        endGrasp = kwargs.get('endGrasp',kwargs.get('grasp',0))
        endJoints = self.getJointsFromPose(armName, endPose, grasp=endGrasp)
        
        self.trajStartGrasp[armName] = startGrasp
        self.trajEndGrasp[armName] = endGrasp
        endPose = raven_util.convertToFrame(tfx.pose(endPose), raven_constants.Frames.Link0)
        self.trajStartJoints[armName] = startJoints
        self.trajEndJoints[armName] = endJoints
    
    def setStartPoseAndEndJoints(self, armName, startPose, endJoints, **kwargs):
        startGrasp = kwargs.get('startGrasp',kwargs.get('grasp',0))
        endGrasp = kwargs.get('endGrasp',kwargs.get('grasp',0))
        startJoints = self.getJointsFromPose(armName, startPose, grasp=startGrasp)
        
        self.trajStartGrasp[armName] = startGrasp
        self.trajEndGrasp[armName] = endGrasp
        startPose = raven_util.convertToFrame(tfx.pose(startPose), raven_constants.Frames.Link0)
        self.trajStartJoints[armName] = startJoints
        self.trajEndJoints[armName] = endJoints
    
    def setStartAndEndPose(self, armName, startPose, endPose, **kwargs):
        startGrasp = kwargs.get('startGrasp',kwargs.get('grasp',0))
        endGrasp = kwargs.get('endGrasp',kwargs.get('grasp',0))
        startJoints = self.getJointsFromPose(armName, startPose, grasp=startGrasp)
        endJoints = self.getJointsFromPose(armName, endPose, grasp=endGrasp)
        
        self.trajStartGrasp[armName] = startGrasp
        self.trajEndGrasp[armName] = endGrasp
        startPose = raven_util.convertToFrame(tfx.pose(startPose), raven_constants.Frames.Link0)
        self.trajStartPose[armName] = startPose
        endPose = raven_util.convertToFrame(tfx.pose(endPose), raven_constants.Frames.Link0)
        self.trajEndPose[armName] = endPose
        self.trajStartJoints[armName] = startJoints
        self.trajEndJoints[armName] = endJoints
    
    def getTrajectoryJointsToPose(self, armName, endPose, startJoints=None, n_steps=50, debug=False, **kwargs):
        self.setStartJointsAndEndPose(armName, startJoints, endPose, **kwargs)
        self.trajSteps[armName] = n_steps
        with self.lock:
            self.trajRequest[armName] = True
        if self.trajEndJoints[armName] is None:
            print armName, endPose, startJoints, kwargs
            raise Exception()
    
    def getTrajectoryPoseToPose(self, armName, startPose, endPose, n_steps=50, **kwargs):
        self.setStartAndEndPose(armName, startPose, endPose, **kwargs)
        self.trajSteps[armName] = n_steps
        with self.lock:
            self.trajRequest[armName] = True
        if self.trajEndJoints[armName] is None:
            print armName, startPose, endPose, kwargs
            raise Exception() 
    
    def getTrajectoryFromPose(self, armName, endPose, startPose=None, endGrasp = None, n_steps=50, block=True, approachDir=None):
        self.waitForState()
        joints1 = self.getCurrentJoints(armName)
        if startPose is None:
            startPose = self.getCurrentPose(armName)
        startGrasp = self.getCurrentGrasp(armName)
        if endGrasp is None:
            endGrasp = startGrasp
        
        if self.errorModel is not None:
            endPose, endPoseOffset = self.errorModel.predictSinglePose(armName, endPose, endPose) # << this is wrong, need true delta

        self.setStartAndEndPose(armName, startPose, endPose, startGrasp=startGrasp, endGrasp=endGrasp)
        self.trajSteps[armName] = n_steps
        with self.lock:
            self.trajRequest[armName] = True
        if self.trajEndJoints[armName] is None:
            print armName, startPose, startGrasp, endPose, endGrasp
            raise Exception()
        
        self.approachDir[armName] = approachDir
        
        self.start_pose_pubs[armName].publish(startPose.msg.PoseStamped())
        self.end_pose_pubs[armName].publish(endPose.msg.PoseStamped())
        
        if block:
            print 'waiting for arm {} traj'.format(armName)
            while self.trajRequest[armName] and not rospy.is_shutdown():
                rospy.sleep(0.05)
            print 'received arm {} traj'.format(armName)

        return self.deltaPoseTraj[armName]   
        
    getPoseTrajectory = getTrajectoryFromPose

    def getFullTrajectoryFromPose(self, armName, endPose, startPose=None, endGrasp = None, n_steps=50, block=True, approachDir=None, speed=None, correctTrajectory=False):
        success = False
        numTries = 0
        while not success and numTries < 10:
            self.waitForState()
            joints1 = self.getCurrentJoints(armName)
            if startPose is None:
                startPose = self.getCurrentPose(armName)
            startGrasp = self.getCurrentGrasp(armName)
            if endGrasp is None:
                endGrasp = startGrasp

            self.setStartAndEndPose(armName, startPose, endPose, startGrasp=startGrasp, endGrasp=endGrasp)
            self.trajSteps[armName] = n_steps
            with self.lock:
                self.trajRequest[armName] = True
            if self.trajEndJoints[armName] is None:
                print armName, startPose, startGrasp, endPose, endGrasp
                raise Exception()
            
            self.approachDir[armName] = approachDir
            
            self.start_pose_pubs[armName].publish(startPose.msg.PoseStamped())
            self.end_pose_pubs[armName].publish(endPose.msg.PoseStamped())
            
            if block:
                print 'waiting for arm {} traj'.format(armName)
                while self.trajRequest[armName] and not rospy.is_shutdown():
                    rospy.sleep(0.05)
                print 'received arm {} traj'.format(armName)
            
            endPoses = [raven_util.endPose(startPose, deltaPose) for deltaPose in self.deltaPoseTraj[armName]]
            success = (endPoses[-1].position.distance(endPose.position) <= 0.005)
            numTries = numTries+1

        if endPoses[-1].position.distance(endPose.position) > 0.005:
            print 'FAIL'

        if correctTrajectory:
            # run a second time to correct (needed the desired approach direction)    
            if self.errorModel is not None:
                #startPoseCorrected, startPoseOffset = self.errorModel.predictSinglePose(armName, startPose, startPose)
                dt = 1
                secondLastPose = endPoses[-2]
                lastPose = endPoses[-1]
                if speed is not None:
                    dt = lastPose.position.distance(secondLastPose.position) / speed

                endPoseCorrected, endPoseOffset = self.errorModel.predictSinglePose(armName, lastPose, secondLastPose, dt) # << this is wrong, need true delta
                print 'End Before', self.poseTraj[armName][n_steps-1]
                

                correctedTraj = self.getFullTrajectoryFromPose(armName, endPoseCorrected, startPose=startPose,
                    endGrasp=endGrasp, n_steps=n_steps, block=block, approachDir=approachDir, correctTrajectory=False)
                
                print
                print armName
                for p in endPoses:
                    print 'Pose', p
                print 'START', startPose
                print 'END', self.poseTraj[armName][n_steps-1]
                print 'ORIG', endPose
                print 'dt', dt
                print
               # if armName == 'R':
                 #   IPython.embed()

                return correctedTraj
        return (self.poseTraj[armName][0], self.deltaPoseTraj[armName])   
        #return self.poseTraj[armName]
    
    def perturbTrajectory(self, armName, poseTraj):
        perturbedPoseList = []

        sigmaPos = 0.001
        sigmaAngle = 1.0

        for i in range(1,len(poseTraj)):
            deltaPos = tfx.random_vector(sigmaPos)
            deltaAngles = tfx.tb_angles(sigmaAngle*np.random.randn(),
                sigmaAngle*np.random.randn(), sigmaAngle*np.random.randn())
            deltaPose = tfx.pose(deltaPos, deltaAngles)
            perturbedPose = tfx.pose(deltaPose.matrix.dot(poseTraj[i].matrix)) 
            perturbedPoseList.append(perturbedPose)
        
        return perturbedPoseList

    def correctPoseTrajectory(self, armName, poseTraj):
        correctedPoseList = []
        
        # create extended pose traj to easily get a delta pose for the first pose
        extendedPoseTraj = copy.copy(poseTraj)
        extendedPoseTraj.insert(0, poseTraj[0])
        
        # correct the poses one by one
        for i in range(1,len(extendedPoseTraj)):
            curPose = extendedPoseTraj[i]
            prevPose = extendedPoseTraj[i-1]
            
            poseGPCorrected = curPose # in case no error model was provided
            poseSysCorrection = curPose

            if self.errorModel is not None:
                poseGPCorrected, poseSysCorrected = self.errorModel.predictSinglePose(armName, curPose, prevPose)
            
            correctedPoseList.append(poseGPCorrected)

        return correctedPoseList
    
    def trajReady(self):
        return not any(self.trajRequest.values())
    
    def waitForTrajReady(self):
        while not self.trajReady() and not rospy.is_shutdown():
            rospy.sleep(.05)
            
    # KINECT-based constraint generation functions
    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        """
        Read points from a L{sensor_msgs.PointCloud2} message.
    
        @param cloud: The point cloud to read from.
        @type  cloud: L{sensor_msgs.PointCloud2}
        @param field_names: The names of fields to read. If None, read all fields. [default: None]
        @type  field_names: iterable
        @param skip_nans: If True, then don't return any point with a NaN value.
        @type  skip_nans: bool [default: False]
        @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
        @type  uvs: iterable
        @return: Generator which yields a list of values for each point.
        @rtype:  generator
        """
        assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        unpack_from = struct.Struct(fmt).unpack_from
    
        if skip_nans:
            if uvs:
                for u, v in uvs:
                    p = unpack_from(data, (row_step * v) + (point_step * u))
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
            else:
                for v in xrange(height):
                    offset = row_step * v
                    for u in xrange(width):
                        p = unpack_from(data, offset)
                        has_nan = False
                        for pv in p:
                            if isnan(pv):
                                has_nan = True
                                break
                        if not has_nan:
                            yield p
                        offset += point_step
        else:
            if uvs:
                for u, v in uvs:
                    yield unpack_from(data, (row_step * v) + (point_step * u))
            else:
                for v in xrange(height):
                    offset = row_step * v
                    for u in xrange(width):
                        yield unpack_from(data, offset)
                        offset += point_step
    
    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        fmt = '>' if is_bigendian else '<'
    
        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length
    
        return fmt
    # END section of point_cloud2.py
    
    def generate_mesh(self, cloud):
        #cloud = cloudprocpy.fastBilateralFilter(cloud, 15, .05) # smooth out the depth image
        big_mesh = cloudprocpy.meshOFM(cloud, 3, .1) # use pcl OrganizedFastMesh to make mesh
        simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, self.decimation_rate) # decimate mesh with VTK function
        return simple_mesh
    # END generate_mesh
    
    def get_xyz_world_frame(self, cloud):
        xyz1 = cloud.to2dArray()
        xyz1[:,3] = 1
        return xyz1.dot(self.T_kinect_to_robot.array.T)[:,:3]
    
    # Plots the point cloud to the env
    # Returns a handle to it
    def plotCloud(self, cloud):
        xyz = self.get_xyz_world_frame(cloud)
        goodinds = np.isfinite(xyz[:,0])
        if isinstance(cloud, cloudprocpy.CloudXYZRGB):
            rgbfloats = cloud.to2dArray()[:,4]
            rgb0 = np.ndarray(buffer=rgbfloats.copy(),shape=(raven_constants.Kinect.height * raven_constants.Kinect.width, 4),dtype='uint8')
            cloud_handle = self.env.plot3(xyz[goodinds,:], 2,(rgb0[goodinds,:3][:,::-1])/255. )
        else:
            cloud_handle = self.env.plot3(xyz[goodinds,:], 2)
        return cloud_handle
    
    # Adds the geometric shape(s) to the env
    # Returns a list of the names of the added KinBodies
    def generateBodies(self, cloud, id):
        # BEGIN addtoenv
        names = []
        if self.geom_type == "mesh":
            mesh = self.generate_mesh(cloud)
            name = "simple_mesh_%i"%id
            mesh_body = mk.create_trimesh(self.env, self.get_xyz_world_frame(mesh.getCloud()), np.array(mesh.getFaces()), name=name)
            mesh_body.SetUserData("bt_use_trimesh", True) # Tell collision checker to use the trimesh rather than the convex hull of it
            names.append(name)
            
        elif self.geom_type == "cd":
            big_mesh = self.generate_mesh(cloud)
            #name = 'big_mesh'
            #verts = self.get_xyz_world_frame(big_mesh.getCloud())
            #mk.create_trimesh(self.env, verts, big_mesh.getTriangles(), name=name)
                
            convex_meshes = cloudprocpy.convexDecompHACD(big_mesh, self.num_cd_components)
            for (i,mesh) in enumerate(convex_meshes):
                name = "mesh_%i_%i" % (id,i)
                verts = self.get_xyz_world_frame(mesh.getCloud())
                mk.create_trimesh(self.env, verts, mesh.getTriangles(), name=name)
                randcolor = np.random.rand(3)
                self.env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetAmbientColor(randcolor)
                self.env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetDiffuseColor(randcolor)
                names.append(name)
        # END  addtoenv   
             
        elif self.geom_type == "spheres":
            raise Exception("don't use spheres--there's a performance issue")
            cloud_ds = cloudprocpy.downsampleCloud(cloud, .04)
            name = "spheres_%i"%id
            mk.create_spheres(self.env, self.get_xyz_world_frame(cloud_ds), .02, name=name)
            names.append(name)
            
        elif self.geom_type == "boxes":
            cloud_ds = cloudprocpy.downsampleCloud(cloud, .04)
            name = "boxes_%i"%id
            mk.create_boxes(self.env, self.get_xyz_world_frame(cloud_ds), .02, name=name)
            names.append(name)
        return names
    
    def load_workspace(self, workspace_file):
        cloud = cloudprocpy.CloudXYZ()
        points = np.load(workspace_file)
        cloud.from3dArray(points)
            
        # add new bodies and cloud first
        self.cd_body_names = self.generateBodies(cloud, self.cloud_id)
    
    # Updates the bodies and point cloud of the openrave environment
    def kinect_callback(self, cloud_msg):
        # only add the constraints ONCE!!
        if self.cloud_id == 0:
            # convert from point cloud message to cloudprocpy.CloudXYZ
            points_gen = self.read_points(cloud_msg, skip_nans=False)
            points = []
            for pt in points_gen:
                pt = list(pt)
                pt.append(1)
                points.append(pt)
            
            # reorganize cloud data to construct a cloud object 
            n_dim = 4 # 3 spatial dimension + 1 for homogenity   
            cloud = cloudprocpy.CloudXYZ()
            xyz1 = np.array(points)
            xyz1 = np.reshape(xyz1, (raven_constants.Kinect.height, raven_constants.Kinect.width, n_dim))
            cloud.from3dArray(xyz1)
            
            #save cloud for next run
            np.save('env_obstacles', xyz1)
            
            # add new bodies and cloud first
            new_cd_body_names = self.generateBodies(cloud, self.cloud_id)
            
            # remove previous bodies and cloud
            for name in self.cd_body_names:
                self.env.Remove(self.env.GetKinBody(name))
            
            self.cd_body_names = new_cd_body_names
            self.cloud_id = (self.cloud_id+1)%2
        #else:
         #   IPython.embed()
            
def testSwitchPlaces(show=True):
    #trajoptpy.SetInteractive(True)
    
    rospy.init_node('testSwitchPlaces',anonymous=True)
    rp = RavenPlanner(raven_constants.Arm.Both, thread=True)
    
    #rightCurrPose = tfx.pose([-0.068,-0.061,-0.129],tfx.tb_angles(-139.6,88.5,111.8),frame=raven_constants.Frames.Link0)
    #leftCurrPose = tfx.pose([-.072,-.015,-.153],tfx.tb_angles(43.9,78.6,100.9),frame=raven_constants.Frames.Link0)
    
    
    leftCurrPose = raven_util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[raven_constants.Arm.Left]),raven_constants.Frames.Link0)
    rightCurrPose = raven_util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[raven_constants.Arm.Right]),raven_constants.Frames.Link0)


    rp.getTrajectoryPoseToPose(raven_constants.Arm.Left, leftCurrPose, rightCurrPose, n_steps=50)
    rp.getTrajectoryPoseToPose(raven_constants.Arm.Right, rightCurrPose, leftCurrPose, n_steps=50)
    
    #rp.getPoseTrajectory(raven_constants.Arm.Left, rightCurrPose, n_steps=50)
    #rp.getPoseTrajectory(raven_constants.Arm.Right, leftCurrPose, n_steps=50)
    #rp.getPoseTrajectory(raven_constants.Arm.Left, leftCurrPose+[0.01,0,0], n_steps=50)
    #rp.getPoseTrajectory(raven_constants.Arm.Right, rightCurrPose-[0.01,0,0], n_steps=50)

    
    print 'waiting'
    rp.waitForTrajReady()
    print 'woooooooo'
    
    if rospy.is_shutdown():
        return
    
    #IPython.embed()
    
    if not show:
        return
    rp.env.SetViewer('qtcoin')
    
    leftPoseTraj = rp.poseTraj[raven_constants.Arm.Left]
    rightPoseTraj = rp.poseTraj[raven_constants.Arm.Right]
    
    for left, right in zip(leftPoseTraj,rightPoseTraj):
        if rospy.is_shutdown():
            break
        rp.updateOpenraveJoints('L', rp.getJointsFromPose('L', left, rp.getCurrentGrasp('L')), grasp=rp.getCurrentGrasp('L'))
        rp.updateOpenraveJoints('R', rp.getJointsFromPose('R', right, rp.getCurrentGrasp('R')), grasp=rp.getCurrentGrasp('R'))
        rospy.loginfo('Press enter to go to next step')
        raw_input()
        
    return
    
    leftTraj = rp.jointTraj[raven_constants.Arm.Left]
    rightTraj = rp.jointTraj[raven_constants.Arm.Right]
    
    for left, right in zip(leftTraj,rightTraj):
        if rospy.is_shutdown():
            break
        rp.updateOpenraveJoints('L', left)
        rp.updateOpenraveJoints('R', right)
        rospy.loginfo('Press enter to go to next step')
        raw_input()
    
    #IPython.embed()
    
def testFromAbove(show=True):
    trajoptpy.SetInteractive(True)
    
    rospy.init_node('testFromAbove',anonymous=True)
    rp = RavenPlanner([raven_constants.Arm.Left, raven_constants.Arm.Right], thread=True)
    
    leftCurrPose = raven_util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[raven_constants.Arm.Left]),raven_constants.Frames.Link0)
    rightCurrPose = raven_util.convertToFrame(tfx.pose([0,0,0],frame=rp.toolFrame[raven_constants.Arm.Right]),raven_constants.Frames.Link0)

    rp.getTrajectoryFromPose(raven_constants.Arm.Left, leftCurrPose-[0,0.05,0], n_steps=50, approachDir=np.array([0,0,1]), block=False)
    rp.getTrajectoryFromPose(raven_constants.Arm.Right, rightCurrPose-[0,0.05,0], n_steps=50, approachDir=np.array([0,0,1]))
    
    #IPython.embed()
    
    print 'waiting'
    rp.waitForTrajReady()
    print 'woooooooo'
    
    if rospy.is_shutdown():
        return
    
    #IPython.embed()
    
    if not show:
        return
    rp.env.SetViewer('qtcoin')
    
    leftPoseTraj = rp.poseTraj[raven_constants.Arm.Left]
    rightPoseTraj = rp.poseTraj[raven_constants.Arm.Right]
    

    
    leftTraj = rp.jointTraj[raven_constants.Arm.Left]
    rightTraj = rp.jointTraj[raven_constants.Arm.Right]
    
    for right in rightTraj:
        if rospy.is_shutdown():
            break
        rp.updateOpenraveJoints('L', left)
        rp.updateOpenraveJoints('R', right)
        rospy.loginfo('Press enter to go to next step')
        raw_input()
    
    IPython.embed()
    
def testRavenCpp():
    trajoptpy.SetInteractive(True)
    
    rospy.init_node('testRavenCpp',anonymous=True)
    
    arm = raven_constants.Arm.Right
    
    rp = RavenPlanner([arm],thread=True)
    
    manip = rp.robot.GetManipulators()[1]
    
    #rp.env.SetViewer('qtcoin')
    
    #endJoints = [0.5672114866971969, 1.5318772029876708, -0.12326233461499214, -0.18351784765720369, -0.55512279719114299, -0.10556840419769287]
    #rp.robot.SetActiveDOFValues(endJoints)
    
    
    startJoints = [0.15336623787879944, 1.5377614498138428, -0.11949782818555832, -0.23602290451526642, -0.14983920753002167, -0.10172462463378906]
    rp.robot.SetActiveDOFValues(startJoints)
    
    #print('press enter')
    #raw_input()
    
    startPose = tfx.pose(raven_util.openraveTransformFromTo(rp.robot, np.eye(4), 'tool_R', '0_link'),frame=raven_constants.Frames.Link0)
    endPose = tfx.pose(startPose + [0,-.1,0])
    
    raveStartEE = manip.GetEndEffectorTransform()
    raveEndEE = np.array(raveStartEE)
    raveEndEE[0,3] += .1
    
    g = []
    g += raven_util.plot_transform(rp.env, raveStartEE)
    g += raven_util.plot_transform(rp.env, raveEndEE)
    
    box = rave.RaveCreateKinBody(rp.env,'')
    box.SetName('testbox')
    box.InitFromBoxes(np.array([[0,0,0,0.01,0.04,0.015]]),True)
    #code.interact(local=locals())
    ee = .5*(raveStartEE + raveEndEE)
    ee[:3,:3] = np.identity(3)
    box.SetTransform(ee)
    rp.env.Add(box,True)
    
    IPython.embed()
    
    rp.getTrajectoryFromPose(arm, endPose, startPose=startPose, n_steps=50)
    
    print('waiting...')
    rp.waitForTrajReady()
    print('traj ready!')

    
    rightTraj = rp.jointTraj[raven_constants.Arm.Right]
    
    for right in rightTraj:
        if rospy.is_shutdown():
            break
        rp.updateOpenraveJoints('R', right)
        rospy.loginfo('Press enter to go to next step')
        raw_input()
    
    rospy.spin()
    

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--show',action='store_true',default=False)
    
    args = parser.parse_args()
    testSwitchPlaces(**vars(args))
    #testFromAbove(**vars(args))
    #testRavenCpp()