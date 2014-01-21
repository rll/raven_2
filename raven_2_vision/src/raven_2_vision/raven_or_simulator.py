"""Shows how to get all 6D IK solutions.
"""
#!/usr/bin/env python

# Stuff that could be wrong - code template was borrowed from Jonathan
#TODO: make dictionary mapping from OR joints to ROS joints and vice versa
#TODO: check joint limits
#TODO: check raven_2/raven_2_trajectory/raven_planner.py
#TODO: something wrong with the pose calculation from joint angles . .. 

import roslib
roslib.load_manifest('raven_2_vision')

#====== GENERAL ==========#
import math
import numpy as np 
from numpy import *
from numpy.linalg import *
from optparse import OptionParser
import threading
import time

import os
import rospy

import matplotlib as mpl
import matplotlib.pyplot as plt

import IPython

#====== ROS ==============#
#import tf
#import tf.transformations as tft
from raven_2_msgs.msg import RavenCommand
from raven_2_msgs.msg import Constants
from raven_2_control.kinematics import *
from raven_2_utils import raven_constants
from raven_2_utils import raven_util
import tfx
#import roslib; roslib.load_manifest("raven_2_teleop")
#import rospy
#from raven_2_msgs.msg import *
#from geometry_msgs.msg import Pose, Point, Quaternion
#from raven_2_trajectory.srv import RecordTrajectory, RecordTrajectoryResponse

#====== OPENRAVE =========#
import openravepy as rave

"""

BASE_FRAME = '/0_link'
END_EFFECTOR_FRAME_PREFIX = '/tool_'
END_EFFECTOR_FRAME_SUFFIX = ['L','R']
SIDE_ACTIVE = [True,False]
SIDE_NAMES = ['L','R']
SIDE_NAMES_FRIENDLY = ['left','right']

"""

#========================= CONSTANTS ============================#
ARM = "ONE_ARM" # or "TWO_ARM"
MODEL_NAME = "myRavenSimulator.xml"
SENSOR_PARENT = "0_link"
DEBUG = True

# wtf is this
X_SCALE=0.0003
Y_SCALE=0.0003
Z_SCALE=0.0006

SHOULDER   =0
ELBOW      =1
Z_INS      =2
TOOL_ROT   =4
WRIST      =5
GRASP1     =6
GRASP2     =7
YAW        =8
GRASP      =9

ACTIVE_THRESHOLD = 5

JOINTS_ARRAY_INDICES = [SHOULDER, ELBOW, Z_INS, TOOL_ROT, WRIST, GRASP1, GRASP2, YAW, GRASP]
ROS_TO_L_OR = {0:2,
               1:3,
               2:4,
               4:5,
               5:6,
               6:8,
               7:10,
               8:7,
               9:9
               } 
ROS_TO_R_OR = {}
L_OR_TO_ROS = {2:0, # shoulder_L --> shoulder
               3:1, # elbow_L --> elbow
               4:2, # insertion_L--> z_ins
               5:4, # tool_roll_L--> tool_rot
               6:5, # wrist_joint_L--> wrist
               8:6, # grasper_joint_1_L --> grasp1
               10:7, #grasper_joint_2_L --> grasp2
               7:8, #grasper_yaw_L --> yaw
               9:9, #grasper1_tip_L --> grasp
               }
R_OR_TO_ROS = {}

prevFrame = None
currFrame = None
FramesLock = threading.Lock()

SHOULDER_MIN_LIMIT =(   0.0 * DEG2RAD)
SHOULDER_MAX_LIMIT =(  90.0 * DEG2RAD)
ELBOW_MIN_LIMIT =(  45.0 * DEG2RAD)
ELBOW_MAX_LIMIT =( 135.0 * DEG2RAD)

Z_INS_MIN_LIMIT =(-0.230)
Z_INS_MAX_LIMIT =( 0.010)

TOOL_ROLL_MIN_LIMIT =(-182.0 * DEG2RAD)
TOOL_ROLL_MAX_LIMIT =( 182.0 * DEG2RAD)
TOOL_WRIST_MIN_LIMIT =(-75.0 * DEG2RAD)
TOOL_WRIST_MAX_LIMIT =( 75.0 * DEG2RAD)

TOOL_GRASP_LIMIT = 89.0 * DEG2RAD
TOOL_GRASP1_MIN_LIMIT = (-TOOL_GRASP_LIMIT)
TOOL_GRASP1_MAX_LIMIT =   TOOL_GRASP_LIMIT
TOOL_GRASP2_MIN_LIMIT = (-TOOL_GRASP_LIMIT)
TOOL_GRASP2_MAX_LIMIT =   TOOL_GRASP_LIMIT

# inverse transform from camera to 0 link
ROBOT_TRANSFORM = np.array([[-0.9996,    0.0196,    0.0204,   -0.0443],
                            [-0.0086,    0.4763,   -0.8793,   -0.0933],
                            [-0.0270,   -0.8791,   -0.4759,    0.2272],
                            [0,         0,         0,    1.0000]])

# intrisics for left BC
CAMERA_INTRINSICS = np.array([[1671.22765, 0.0, 667.47155],
                              [0.0, 1667.28134, 494.89699],
                              [0.0, 0.0, 1.0]])
# left BC image size
IM_W = 1280
IM_H = 960
DOWNSAMPLE = 4 # downsample rate in disparity image node
CAMERA_INTRINSICS = CAMERA_INTRINSICS / DOWNSAMPLE
IM_W = IM_W / DOWNSAMPLE
IM_H = IM_H / DOWNSAMPLE
        

#========================= RAVEN CONTROLLER CLASS ==========================================================#
class RavenSimulator:
    
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
    
    def __init__(self, arm_mode=ARM, arm_names=raven_constants.Arm.Left, x_scale=X_SCALE, y_scale=Y_SCALE, z_scale=Z_SCALE, initPose=None, initGrasp=0, initJoints = None, frame=None, relative_orientation=False, camera_frame=False):
        if isinstance(arm_names,basestring):
            arm_names = [arm_names]
        self.arm_names = sorted(arm_names)
        
        self.raven_pub = rospy.Publisher('raven_command', RavenCommand)
        self.arm_mode = arm_mode
        self.x_scale = x_scale
        self.y_scale = y_scale
        self.z_scale = z_scale 
        self.active = False
        
        self.initPose = initPose
        self.initGrasp = initGrasp
        self.initJoints = initJoints
 
        if self.initJoints is not None:
            self.initGrasp = self.initJoints[GRASP]
        
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
        
        self.env = rave.Environment()
        ravenModelFile = os.path.join(roslib.packages.get_pkg_subdir('RavenDebridement', 'models'), MODEL_NAME)
        self.env.Load(ravenModelFile)
        self.robot = self.env.GetRobots()[0]
        
        # transform the robot such that the sensor parent is at the origin
        link = self.robot.GetLink(SENSOR_PARENT)
        link_tf = link.GetTransform()
        link_inv_tf = np.linalg.inv(link_tf)
        
        activeDOFs = []
        for armName in self.arm_names:
            self._init_arm(armName)
            for jointType in self.raveJointTypes[armName]:
                if jointType >= 0:
                    activeDOFs += [jointType]
        self.robot.SetActiveDOFs(activeDOFs)
        
        # configure laser scanner for sensing
        robot_tf = ROBOT_TRANSFORM.dot(link_inv_tf)
        self.robot.SetTransform(robot_tf)
        self.sensor = self.env.GetSensors()[0]
        self.sensor.Configure(rave.Sensor.ConfigureCommand.PowerOn)
        self.sensor.Configure(rave.Sensor.ConfigureCommand.RenderDataOn)
        self.olddata = self.sensor.GetSensorData(rave.Sensor.Type.Laser)
        #self.env.SetViewer('qtcoin')
        #self.updateOpenraveJoints('L', self.initJoints, self.initGrasp)
        #depth_im = self.getSensorData(disp=disp)


    def _init_arm(self, arm_name):
        if arm_name == raven_constants.Arm.Left:
            self.invKinArm[arm_name] = Constants.ARM_TYPE_GOLD
            self.toolFrame[arm_name] = raven_constants.OpenraveLinks.LeftTool
            self.manipName[arm_name] = 'left_arm'
        else:
            self.invKinArm[arm_name] = Constants.ARM_TYPE_GREEN
            self.toolFrame[arm_name] = raven_constants.OpenraveLinks.RightTool
            self.manipName[arm_name] = 'right_arm'
        self.robot.SetActiveManipulator(self.manipName[arm_name])
        self.manip[arm_name] = self.robot.GetActiveManipulator()
        self.manipJoints[arm_name] = self.robot.GetJoints(self.manip[arm_name].GetArmJoints())

        self.raveJointNames[arm_name] = ['{0}_{1}'.format(name, arm_name[0].upper()) for name in self.raveJointNamesPrefixes]

        self.raveJointTypes[arm_name] = [self.robot.GetJointIndex(name) for name in self.raveJointNames[arm_name]]
        self.raveJointTypesToRos[arm_name] = dict((rave,ros) for rave,ros in zip(self.raveJointTypes[arm_name], self.rosJointTypes))
        self.rosJointTypesToRave[arm_name] = dict((ros,rave) for ros,rave in zip(self.rosJointTypes, self.raveJointTypes[arm_name]))
        
        self.raveGrasperJointNames[arm_name] = ['grasper_joint_1_{0}'.format(arm_name[0].upper()), 'grasper_joint_2_{0}'.format(arm_name[0].upper())]
        self.raveGrasperJointTypes[arm_name] = [self.robot.GetJointIndex(name) for name in self.raveGrasperJointNames[arm_name]]
        
        self.updateOpenraveJoints(arm_name, self.initJoints, grasp=self.initGrasp)

    #========================== OPEN RAVE COMMANDS ================================#
    def updateOpenraveJoints(self, armName, joints1, grasp=None):
        """
        Updates the openrave raven model using rosJoints

        rosJoints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Updates based on: shoulder, elbow, insertion,
                          rotation, pitch, yaw, grasp (self.currentGrasp)
        """
        rosJoints = joints1
        
        if grasp is None:
            grasp = pi/2.
        
        raveJointTypes = []
        jointPositions = []
        if rosJoints is not None:
            for rosJointType, jointPos in rosJoints.items():
                if self.rosJointTypesToRave[armName].has_key(rosJointType):
                    raveJointType = self.rosJointTypesToRave[armName][rosJointType]
                    if raveJointType > 0:
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
           #  IPython.embed()
            self.robot.SetJointValues(jointPositions, raveJointTypes)
            return True
        return False


    def publishORCommand(self, joints, joints_array_indices):
        """ Publish a command to the robot; sets joints directly"""
        if joints != None and joints_array_indices != None:
            self.robot.SetDOFValues(joints, joints_array_indices)

    def getORCommand(self, p, c, grip):
        """ Calculates joints based off of input from the leap motion """
        joints, joints_array_indices = self.calculateNewPose(p,c,grip)
        return joints, joints_array_indices


    def getExpectedMeasurement(self, pose, grasp=None, arm=raven_constants.Arm.Left, disp=False):
        """ Transforms the simulated Raven by the expected pose and raycasts to get the expected measurement"""
        if grasp is None:
            grasp = self.initGrasp
        
        joints = invArmKin(arm, pose, grasp)
        success = self.updateOpenraveJoints(arm, joints, grasp)
        if success:
            start = time.clock()
            depth_im = self.getSensorData(disp=disp)
            stop = time.clock()
            #print "Simulator query time (sec):", stop-start
            return depth_im
        return None
        
    def getSensorData(self, disp=False):
        max_range = 1.0
        
        olddata = self.sensor.GetSensorData(self.sensor.Type.Laser)
        while True:
            data = self.sensor.GetSensorData(self.sensor.Type.Laser)
            if data.stamp != olddata.stamp:
                break
            time.sleep(0.005)
        
        # get points from data buffer    
        points = data.ranges
        invalid_inds = abs((points**2).sum(axis=1)-max_range**2)<1e-3
        points[invalid_inds,2] = 0
        
        points_proj = points.T
        depths = points_proj[2,:]
        
        # project points into imaging plane
        K = CAMERA_INTRINSICS
        points_proj = K.dot(points_proj)
        points_proj = points_proj / depths
        points_proj = np.round(points_proj)
        
        # place valid points in image
        depth_im = np.zeros((IM_H, IM_W))
        for i in range(points_proj.shape[1]):
            p = points_proj[:,i]
            
            # make sure point lies within image, then place in buffer
            if p[0] >= 0 and p[0] < IM_W and p[1] >= 0 and p[1] < IM_H:
                #   IPython.embed()
                depth_im[p[1], p[0]] = depths[i]
        
        if disp:
            plt.imshow(depth_im)
            plt.show()
        return depth_im

    #====================== OPEN RAVE HELPER FUNCTIONS =============================#
    def calculateJoints(self, pose, arm, grasp=0.0):
        #pose = ROBOT_TRANSFORM.dot(pose)
        try:
            result_joints_dict = invArmKin(arm, pose, grasp, False)
            if result_joints_dict is not None:
                joints = []
                joints_array_indices = []
                for key in sorted(result_joints_dict.keys()):
                    joints.append(result_joints_dict[key])
                    joints_array_indices.append(ROS_TO_L_OR[key])
                return True, joints, joints_array_indices
        except ValueError as value_error:
            print value_error
            print "ILLEGAL VALUE PASSED TO INVKIN SOLVER"
            return False, None, None
        print "INVERSE KINEMATICS FAILED"
        return False, None, None
    
#========== HELPER FUNCTIONS ==========#
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

#================= MAIN ================#
def main():
    # Create a sample simulator
    print "1"
    simulator = RavenSimulator(initGrasp=1.2)
    
    print "2"
    rospy.loginfo('Press enter to move ')
    raw_input()
 
    cur_pose = simulator.prevPose
    pose = cur_pose + np.array([-0.01, -0.01, -0.01])
    #IPython.embed()
    simulator.getExpectedMeasurement(pose, grasp=1.2, disp=True)
    
    rospy.loginfo('Press enter to move ')
    raw_input()
    pose = pose + np.array([0.01, 0.01, 0.01])
    simulator.getExpectedMeasurement(pose)
    
    rospy.loginfo('Press enter to quit ')
    raw_input()

if __name__ == "__main__":
    main()
