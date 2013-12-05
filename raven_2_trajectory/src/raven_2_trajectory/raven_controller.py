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
import IPython

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
        if len(stages) > 0:
            stage = stages[0]
        #for stage in stages:
            stageBreaks.append(stage.duration + stageBreaks[-1])
        return stageBreaks

    @property
    def cb(self):
        cb_code = marshal.loads(self.cb_string)
        cb = types.FunctionType(cb_code, globals(), argdefs=self.cb_defaults, closure=self.cb_closure)
        return cb

class RavenController():
    def __init__(self, arm, closedGraspValue=0.,defaultPoseSpeed=.01):
        self.arm = arm

        self.stopRunning = threading.Event()
        self.stopRunning.set()
        # ADDED, initializes the rest
        
        self.stages = []
        self.stagesQueue = mp.Queue()

        # cm/sec
        self.defaultPoseSpeed = defaultPoseSpeed
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
        
		
        self.queue = mp.Queue()
        self.clearStageQueue = mp.Queue()
            
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
        
        
        self.thread = mp.Process(target=self.run, args=(self.queue, self.pubQueue, self.clearStageQueue))
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
            
        while not self.clearStageQueue.empty():
            val = self.clearStageQueue.get()
            if type(val) == dict:
                if val.has_key('clearStages'):
                    if val['clearStages']:
                        self.clearStages()
                if val.has_key('popStage'):
                    if val['popStage']:
                        self.popStage()

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

    def stop(self):
        """
        Sets flag to stop executing trajectory
        """
        #self.stopRunning.set()
        self.stopRunning = True
        self.queue.put({'stopRunning':self.stopRunning})
        return True


    def clearStages(self):
        """
        If playing, this effectively pauses the raven
        """
        self.stages = []
        self.queue.put({'stages':self.stages})
        
    def popStage(self):
        """
        Remove the first stage
        """
        self.stages.pop(0)
        self.queue.put({'updatedStages':self.stages})

    def start(self):
        """
        Intended use is to call play once at the beginning
        and then add stages to move it
        """
        self.stopRunning = False
        self.queue.put({'stopRunning':self.stopRunning})
        return True
    
        if self.thread is None or not self.thread.is_alive():
            self.thread = multiprocessing.Process(target=self.run)
            self.thread.daemon = True
            self.thread.start()
            

        return True

    def run(self, queue, pubQueue, clearStageQueue):
        rate = rospy.Rate(50)
        
        cmd = None
        header = Header()
        header.frame_id = raven_constants.Frames.Link0


        startTime = rospy.Time.now()
        numStages = 0
        success = None
        stageIndex = -1

        cmd = self.ravenPauseCmd

        # ADDED
        #self.stopRunning.clear()
        stopRunning = False
        stages = list()
        runlevel = 0
        
        lastStageIndex = -1
        while True:
            rate.sleep()
            
            while not queue.empty():
                val = queue.get()
                if type(val) == dict:
                    if val.has_key('stopRunning'):
                        stopRunning = val['stopRunning']
                    if val.has_key('stages'):
                        stages = val['stages']
                        print "NEW STAGES", self.arm
                        if stageIndex == -1 and len(stages) > 0:
                            print "FRESH", self.arm
                            startTime = rospy.Time.now()
                            stageIndex = 0
                    if val.has_key('updatedStages'):
                        stages = val['updatedStages']
                    if val.has_key('runlevel'):
                        runlevel = val['runlevel']
            
            if runlevel == 0:
                #rospy.loginfo('Raven in E-STOP')
                success = False
                continue


            print "STAGES", stages, self.arm
            if stopRunning or rospy.is_shutdown():
                success = True
                break
            if stageIndex > -1 and len(stages) > 0:
                # print stageIndex
                stage = stages[stageIndex]
                now = rospy.Time.now()
                durFromStart = now - startTime
                
                header.stamp = now
                cmd.header = header
                
                if durFromStart > stage.duration:
                    print "INCREMENTING STAGE", self.arm
                    stageIndex = stageIndex + 1
                    startTime = rospy.Time.now()
                    if stageIndex >= len(stages):
                        print "CLEARING STAGES", self.arm
                        clearStageQueue.put({'clearStages' : True})
                        stageIndex = -1
                    continue
               
                if stage.duration.is_zero():
                    t = 1
                else:
                    t = min((durFromStart).to_sec() / stage.duration.to_sec(), 1)
                print "Sending command", self.arm, t, stage.name, stage.duration, stageIndex
                cmd = RavenCommand()
                cmd.pedal_down = True
            
                stage.cb(cmd,t)
                """
            now = rospy.Time.now()
            durFromStart = now - startTime
               
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
                    
                # find new stage
                for idx,stageBreak in enumerate(stageBreaks):
                    if stageBreak > durFromStart:
                        stageIndex = idx-1
                        break
                else:
                    clearStageQueue.put({'clearStages' : True})
                    continue
                # get the index of the stage to execute
                stageIndex = min(stageIndex,lastStageIndex + 1)
                lastStageIndex = stageIndex
                stage = stages[stageIndex]
                    
                if stage.duration.is_zero():
                    t = 1
                else:
                    t = (durFromStart - stageBreaks[stageIndex]).to_sec() / stage.duration.to_sec()
                print "Sending command", t
                cmd = RavenCommand()
                cmd.pedal_down = True
                
                stage.cb(cmd,t)
                   """
            else:
                # no stages
                #cmd = self.ravenPauseCmd
                # if last command was gripper, then ignore
                #if cmd.arms[0].tool_command.grasp_option != ToolCommand.GRASP_OFF:
                #    cmd = self.ravenPauseCmd
                pass
			
            pubQueue.put(cmd)
            #self.pubCmd.publish(cmd)
        print 'Raven Controller Thread Exited...'
        return success
    

    ############################
    # Commanding the raven arm #
    ############################

    def addStage(self, name, duration, cb):
        self.stages.append(Stage(name,duration,cb))
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
        def fn(cmd,t, startGrasp=startGrasp, grasp=grasp, closedGraspValue=self.closedGraspValue, arm=self.arm):
            cmdGraspValue = (startGrasp) + (grasp - (startGrasp - closedGraspValue))*t
            RavenController.addArmGraspCmd(cmd, arm, grasp=cmdGraspValue, graspOption=ToolCommand.GRASP_SET_NORMALIZED)
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
            

        

        
def test_gripperMove():
    rospy.init_node('raven_controller',anonymous=True)
    leftArm = RavenController(raven_constants.Arm.Left)
    rospy.sleep(2)

    rospy.loginfo('Press enter to start')
    raw_input()

    leftArm.start()

    leftArm.setGripper(0.0)
    
    rospy.sleep(5)
    
    leftArm.setGripper(1.2)

    startPose = leftArm.currentPose
    endPose = raven_util.endPose(startPose, tfx.pose([0.02, -0.02, -0.02]), startPose.frame)
    leftArm.goToPose(endPose)
    
    rospy.loginfo('Press enter to stop')
    raw_input()

    leftArm.stop()

    rospy.loginfo('Press enter to exit')
    raw_input()



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
    #test_goToPoseUsingJoints()
    test_gripperMove()