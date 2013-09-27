#!/usr/bin/env python

"""
Adapted from Ben Kehoe's trajectory_player
"""

import roslib
roslib.load_manifest('raven_2_trajectory')
import rospy
from math import *

from raven_2_msgs.msg import * 
from std_msgs.msg import Header
from geometry_msgs.msg import *

import numpy as np


import tfx

import threading
import multiprocessing as mp

from raven_2_utils import raven_util
from raven_2_utils import raven_constants

import marshal, types

class Stage(object):
    def __init__(self,name,duration,cb):
        self.name = name
        self.duration = rospy.Duration(duration)
        self.cb_string = marshal.dumps(cb.func_code)
        self.cb_closure = cb.func_closure
        self.cb_defaults = cb.func_defaults
		
    @staticmethod
    def stageBreaks(stages):
        stageBreaks = [rospy.Duration(0)]
        for stage in stages:
            stageBreaks.append(stage.duration + stageBreaks[-1])
        return stageBreaks

    @property
    def cb(self):
        cb_code = marshal.loads(self.cb_string)
        cb = types.FunctionType(cb_code, globals(), argdefs=self.cb_defaults, closure=self.cb_closure)
        return cb

class RavenController():
    def __init__(self, arm, closedGraspValue=0.):
        self.arm = arm

        self.stopRunning = threading.Event()
        print self.stopRunning.isSet()
        self.stopRunning.set()
        print self.stopRunning.isSet()

        # ADDED, initializes the rest
        self.reset()
		
        self.queue = mp.Queue()
            
        self.pubCmd = rospy.Publisher('raven_command', RavenCommand)
        self.pubQueue = mp.Queue()
		
        self.stateSub = rospy.Subscriber('raven_state',raven_2_msgs.msg.RavenState,self._stateCallback)
	
        
        #self.thread = None
        self.header = Header()
        self.header.frame_id = raven_constants.Frames.Link0

        
        # actual grasp value when gripper is closed (in )
        self.closedGraspValue = closedGraspValue*(pi/180.)

        #################
        # PAUSE COMMAND #
        #################
        header = Header()
        header.frame_id = raven_constants.Frames.Link0
        header.stamp = rospy.Time.now()
    
        # create the tool pose
        toolCmd = ToolCommand()
        toolCmd.pose = tfx.pose([0,0,0]).msg.Pose()
        toolCmd.pose_option = ToolCommand.POSE_RELATIVE
        toolCmd.grasp_option = ToolCommand.GRASP_OFF
    
        
        # create the arm command
        armCmd = ArmCommand()
        armCmd.tool_command = toolCmd
        armCmd.active = True
        
        # create the raven command
        ravenPauseCmd = RavenCommand()
        ravenPauseCmd.arm_names.append(self.arm)
        ravenPauseCmd.arms.append(armCmd)
        ravenPauseCmd.pedal_down = True
        ravenPauseCmd.header = header
        ravenPauseCmd.controller = Constants.CONTROLLER_CARTESIAN_SPACE

        self.ravenPauseCmd = ravenPauseCmd
        
        
        self.thread = mp.Process(target=self.run, args=(self.queue, self.pubQueue))
        self.thread.daemon = True
        self.thread.start()
        
    
    def _stateCallback(self, msg):
        self.currentState = msg
        self.queue.put({'runlevel':self.currentState.runlevel})
        self.runlevel.value = self.currentState.runlevel
        for arm in msg.arms:
            if arm.name == self.arm:
                self.currentPose = tfx.pose(arm.tool.pose, header=msg.header)
                self.currentGrasp = arm.tool.grasp

                self.currentJoints = dict((joint.type, joint.position) for joint in arm.joints)
                
        if not self.pubQueue.empty():
            self.header.stamp = rospy.Time.now()
            cmd = self.pubQueue.get()
            cmd.header = self.header
            self.pubCmd.publish(cmd)

    def getCurrentJoints(self):
        """
        Returns is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians
        """
        return self.currentJoints


    ###############################################
    # start, stop, and resetting of the raven arm #
    ###############################################

    def reset(self):
        """
        Resets player, allows for reuse.
        Never needs to be called by user, is automatically
        called when start is called.
        """
        self.stages = []
        self.stagesQueue = mp.Queue()

        # cm/sec
        self.defaultPoseSpeed = .01
        # rad/sec
        #self.defaultJointSpeed = pi/30

        self.defaultJointSpeed = {Constants.JOINT_TYPE_SHOULDER      : pi/16,
                                  Constants.JOINT_TYPE_ELBOW         : pi/25,
                                  Constants.JOINT_TYPE_INSERTION     : pi/512,
                                  Constants.JOINT_TYPE_ROTATION      : pi/4,
                                  Constants.JOINT_TYPE_PITCH         : pi/16,
                                  Constants.JOINT_TYPE_GRASP_FINGER1 : pi/16,
                                  Constants.JOINT_TYPE_GRASP_FINGER2 : pi/16}

		
        self.currentState = None
        self.runlevel = mp.Value('d',0.0)
        self.currentPose = None
        self.currentGrasp = None
        self.currentJoints = None
		
        # ADDED
        #self.stopRunning.clear()


    def stop(self):
        """
        Sets flag to stop executing trajectory

        Returns False if times out, otherwise True
        """
        #self.stopRunning.set()
        self.stopRunning = True
        self.queue.put({'stopRunning':self.stopRunning})
        return True
    
        if self.thread is None:
            return True
        
        self.stopRunning.set()

        rospy.sleep(.1)
        
        self.thread.terminate()
        self.thread = None

#         rate = rospy.Rate(50)
#         timeout = raven_util.Timeout(999999)
#         timeout.start()
#         while not timeout.hasTimedOut():
#             if not self.thread.is_alive():
#                 return True
# 
#             rate.sleep()

        return False


    def clearStages(self):
        """
        If playing, this effectively pauses the raven
        """
        self.stages = []
        self.queue.put({'stages':self.stages})

    def start(self):
        """
        Intended use is to call play once at the beginning
        and then add stages to move it
        """
        #self.stopRunning.clear()
        self.stopRunning = False
        self.queue.put({'stopRunning':self.stopRunning})
        return True
    
        if self.thread is None or not self.thread.is_alive():
            #self.reset()
            
            self.thread = multiprocessing.Process(target=self.run)
            self.thread.daemon = True
            self.thread.start()
            
#           self.thread = threading.Thread(target=self.run)
#           self.thread.setDaemon(True)
#           self.thread.start()

        return True

    def run(self, queue, pubQueue):
        rate = rospy.Rate(50)
        
        cmd = None
		
#         print 'waiting for currentState'
#         while self.currentState is None and not rospy.is_shutdown():
#             rate.sleep()
#             print self.currentState
#             print self.currentState is None
#         print 'found currentState'
#         
#         if self.currentState.runlevel == 0:
#             rospy.loginfo("Raven in E-STOP, waiting")
#             while self.currentState.runlevel == 0 and not rospy.is_shutdown():
#                 rate.sleep()
# 		
        header = Header()
        header.frame_id = raven_constants.Frames.Link0


        startTime = rospy.Time.now()
        numStages = 0
		
        success = None

        cmd = self.ravenPauseCmd

        # ADDED
        #self.stopRunning.clear()
        stopRunning = True
        stages = list()
        runlevel = 0
		
        lastStageIndex = -1
        print 'in rc loop'
        while not rospy.is_shutdown():
            rate.sleep()
            
            while not queue.empty():
                val = queue.get()
                if type(val) == dict:
                    if val.has_key('stopRunning'):
                        stopRunning = val['stopRunning']
                    if val.has_key('stages'):
                        stages = val['stages']
                    if val.has_key('runlevel'):
                        runlevel = val['runlevel']
			
            if runlevel == 0:
                rospy.loginfo('Raven in E-STOP')
                success = False
                continue

            # ADDED
            #print 'stopRunning.isSet: {0}'.format(stopRunning.isSet())
            if stopRunning:
                #self.stopRunning.clear()
                #print 'stopRunning is set'
                success = True
                continue
            #print 'stopRunning is not set'

            #stages = self.stages

            # when a stage appears, set startTime
            if numStages == 0 and len(stages) > 0:
                startTime = rospy.Time.now()

            numStages = len(stages)
            stageBreaks = Stage.stageBreaks(stages)
            
            now = rospy.Time.now()
            
            header.stamp = now
            cmd.header = header

            if numStages > 0:
                durFromStart = now - startTime
                stageIndex = 0
                for idx,stageBreak in enumerate(stageBreaks):
                    if stageBreak > durFromStart:
                        stageIndex = idx-1
                        break
                else:
                    self.clearStages()
                    continue
                stageIndex = min(stageIndex,lastStageIndex + 1)
            
                lastStageIndex = stageIndex
            
                stage = stages[stageIndex]
            
                if stage.duration.is_zero():
                    t = 1
                else:
                    t = (durFromStart - stageBreaks[stageIndex]).to_sec() / stage.duration.to_sec()
            
            
                cmd = RavenCommand()
                cmd.pedal_down = True
            
                stage.cb(cmd,t)
            
            else:
                # no stages
                #cmd = self.ravenPauseCmd
                # if last command was gripper, then ignore
                #if cmd.arms[0].tool_command.grasp_option != ToolCommand.GRASP_OFF:
                #    cmd = self.ravenPauseCmd
                pass
			
            #print 'publish'
            #rospy.loginfo('publishing {0}'.format(cmd))
            pubQueue.put(cmd)
            #self.pubCmd.publish(cmd)
			
            
        return success


    ############################
    # Commanding the raven arm #
    ############################

    def addStage(self, name, duration, cb):
        self.stages.append(Stage(name,duration,cb))
        #import code
        #code.interact(local=locals())
        self.queue.put({'stages':self.stages})

    def goToPose(self, end, start=None, duration=None, speed=None):
        if start == None:
            start = self.currentPose
            if start == None:
                rospy.loginfo('Have not received currentPose yet, aborting goToPose')
                return

        start = tfx.pose(start)
        end = tfx.pose(end)
        
        if duration is None:
            if speed is None:
                speed = self.defaultPoseSpeed
            duration = end.position.distance(start.position) / speed

        def fn(cmd, t, start=start, end=end, arm=self.arm):
            pose = start.interpolate(end, t)
            
            toolPose = pose.msg.Pose()

            # not sure if correct
            cmd.controller = Constants.CONTROLLER_CARTESIAN_SPACE
            RavenController.addArmPoseCmd(cmd, arm, toolPose)

        self.addStage('goToPose', duration, fn)

    def goToJoints(self, endJoints, startJoints=None, duration=None, speed=None):
        """
        joints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        speed is a factor gain compared to default speeds
        """

        if startJoints == None:
            startJoints = self.currentJoints
            if startJoints == None:
                rospy.loginfo('Have not received startJoints yet, aborting goToJoints')
                return

        # if there doesn't exist a start joint for an end joint, return
        for endJointType in endJoints.keys():
            if not startJoints.has_key(endJointType):
                return

        # if there doesn't exist an end joint for a start joint, ignore it
        for startJointType in startJoints.keys():
            if not endJoints.has_key(startJointType):
                del startJoints[startJointType]

        # make sure no pseudo joints are commanded
        pseudoJoints = [Constants.JOINT_TYPE_YAW, Constants.JOINT_TYPE_GRASP]
        for pseudoJoint in pseudoJoints:
            if startJoints.has_key(pseudoJoint):
                del startJoints[pseudoJoint]
            if endJoints.has_key(pseudoJoint):
                del endJoints[pseudoJoint]
                
        # now there should be one-to-one correspondence
        # between startJoints and endJoints

        if duration is None:
            if speed is None:
                speed = self.defaultJointSpeed
            else:
                speed = dict([ (jointType ,speed * defaultSpeed) for jointType, defaultSpeed in self.defaultJointSpeed.items()])
            duration = max([abs((endJoints[jointType]-startJoints[jointType]))/speed[jointType] for jointType in startJoints.keys()])
        
        
        def fn(cmd, t):
            # t is percent along trajectory
            cmd.controller = Constants.CONTROLLER_JOINT_POSITION

            desJoints = dict()
            for jointType in startJoints.keys():
                startJointPos = startJoints[jointType]
                endJointPos = endJoints[jointType]
                desJoints[jointType] = startJointPos + (endJointPos-startJointPos)*t
            
            RavenController.addArmJointCmds(cmd, self.arm, desJoints)


        self.addStage('goToPoseUsingJoints', duration, fn)
        

    ###############################
    # setting the gripper grasp   #
    ###############################

    def openGripper(self,duration=2):
        """
        DEPRECATED
        """
        def fn(cmd,t):
            RavenController.addArmGraspCmd(cmd, self.arm, grasp=1, graspOption=ToolCommand.GRASP_INCREMENT_SIGN)
        self.addStage('Open gripper',duration,fn)
	
    def closeGripper(self,duration=2):
        """
        DEPRECATED
        """
        def fn(cmd,t):
            RavenController.addArmGraspCmd(cmd, self.arm, grasp=-1, graspOption=ToolCommand.GRASP_INCREMENT_SIGN)
        self.addStage('Close gripper',duration,fn)

    def setGripper(self,grasp,closedValue=None,duration=2):
        startGrasp = self.currentGrasp
        def fn(cmd,t):
            cmdGraspValue = (startGrasp) + (grasp - (startGrasp - self.closedGraspValue))*t
            RavenController.addArmGraspCmd(cmd, self.arm, grasp=cmdGraspValue, graspOption=ToolCommand.GRASP_SET_NORMALIZED)
        self.addStage('Set gripper',duration,fn)


    ###############################################
    # static methods for adding to a RavenCommand #
    ###############################################

    @staticmethod
    def addArmCmd(cmd,armName,toolPose=None,poseOption=ToolCommand.POSE_OFF,grasp=0,graspOption=ToolCommand.GRASP_OFF):
        cmd.arm_names.append(armName)

        arm_cmd = ArmCommand()
        arm_cmd.active = True

        tool_cmd = ToolCommand()
        tool_cmd.pose_option = poseOption
        if toolPose is not None:
            tool_cmd.pose = toolPose
            
        tool_cmd.grasp_option = graspOption
        tool_cmd.grasp = grasp

        arm_cmd.tool_command = tool_cmd

        cmd.controller = Constants.CONTROLLER_CARTESIAN_SPACE
        cmd.arms.append(arm_cmd)

    @staticmethod
    def addArmPoseCmd(cmd,armName,toolPose,poseOption=ToolCommand.POSE_ABSOLUTE):
        return RavenController.addArmCmd(cmd, armName, toolPose=toolPose, poseOption=poseOption, graspOption=ToolCommand.GRASP_OFF)

    @staticmethod
    def addArmGraspCmd(cmd,armName,grasp,graspOption=ToolCommand.GRASP_INCREMENT_SIGN):
        return RavenController.addArmCmd(cmd, armName, grasp=grasp, graspOption=graspOption, poseOption=ToolCommand.POSE_OFF)

    @staticmethod
    def addArmJointCmds(cmd,armName,joints):
        """
        joints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians
        """
        cmd.arm_names.append(armName)

        arm_cmd = ArmCommand()
        arm_cmd.active = True

        for jointType, jointPos in joints.items():
            jointCmd = JointCommand()
            jointCmd.command_type = JointCommand.COMMAND_TYPE_POSITION
            jointCmd.value = jointPos

            arm_cmd.joint_types.append(jointType)
            arm_cmd.joint_commands.append(jointCmd)

        cmd.arms.append(arm_cmd)
            

        

        







def test_startstop():
    rospy.init_node('raven_controller',anonymous=True)
    leftArm = RavenController(raven_constants.Arm.Left)
    rospy.sleep(2)

    rospy.loginfo('Press enter to start')
    raw_input()

    leftArm.start()

    rospy.loginfo('Press enter to stop')
    raw_input()

    leftArm.stop()

    rospy.loginfo('Press enter to exit')
    raw_input()

def test_goToPoseUsingJoints():
    rospy.init_node('raven_controller',anonymous=True)
    rightArmController = RavenController(raven_constants.Arm.Right)
    rospy.sleep(2)

    #rightArmController.start()

    rospy.loginfo('Press enter to move')
    raw_input()

    endJointPositions = list((pi/180.0)*np.array([21.9, 95.4, -7.5, 20.4, -29.2, -11.8]))

    rightArmController.goToPoseUsingJoints(tfx.pose([0,0,0]), endJointPositions=endJointPositions)

if __name__ == '__main__':
    #test_startstop()
    test_goToPoseUsingJoints()
