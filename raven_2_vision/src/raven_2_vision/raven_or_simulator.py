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
from raven_2_utils import raven_constants

import matplotlib as mpl
import matplotlib.pyplot as plt

import IPython

#====== ROS ==============#
#import tf
#import tf.transformations as tft
from raven_2_control.kinematics import *
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
MODEL_NAME = "ravenII_2arm_simulator.xml" 
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
    def __init__(self, arm_mode=ARM, x_scale=X_SCALE, y_scale=Y_SCALE, z_scale=Z_SCALE, initPose=None, initGrasp=0, frame=None, relative_orientation=False, camera_frame=False):
        self.raven_pub = rospy.Publisher('raven_command', RavenCommand)
        self.arm_mode = arm_mode
        self.x_scale = x_scale
        self.y_scale = y_scale
        self.z_scale = z_scale 
        self.active = False
        self.configureOREnv(initPose, initGrasp)  

    def updateActive(self, a):
        if self.active != a:
            if a==False:
                print "Controller inactive"
            else:
                print "Controller active"
        self.active = a

    #========================== OPEN RAVE COMMANDS ================================#

    def configureOREnv(self, initPose=None, initGrasp=0):
        """ This function is called once at the begin to set up the openrave environment with the robot """
        self.env = rave.Environment()
        
        ravenModelFile = os.path.join(roslib.packages.get_pkg_subdir('raven_2_params', 'data'), MODEL_NAME)
        self.env.Load(ravenModelFile)
        self.robot = self.env.GetRobots()[0]
        self.manipulators = self.robot.GetManipulators()
        #manip = self.robot.SetActiveManipulator('left_arm')
        self.manip = self.manipulators[0]
        self.manipIndices = self.manip.GetArmIndices()
        allJoints = self.robot.GetJoints()  
        JointValues = [joint.GetValues()[0] for joint in allJoints]

        #These are the starting joints for the left arm; they are halfway between each limit
        leftJoints = []
        if initPose is None:
            leftJoints = [  (SHOULDER_MAX_LIMIT+SHOULDER_MIN_LIMIT)/2.0,
                            (ELBOW_MAX_LIMIT+ELBOW_MIN_LIMIT)/2.0,    
                            #(Z_INS_MAX_LIMIT+Z_INS_MIN_LIMIT)/2.0,
                            -0.1,
                            0,
                            (TOOL_ROLL_MAX_LIMIT+TOOL_ROLL_MIN_LIMIT)/2.0,
                            (TOOL_WRIST_MAX_LIMIT+TOOL_WRIST_MIN_LIMIT)/2.0,
                            (TOOL_GRASP1_MAX_LIMIT+TOOL_GRASP1_MIN_LIMIT)/2.0,
                            (TOOL_GRASP2_MAX_LIMIT+TOOL_GRASP2_MIN_LIMIT)/2.0,
                            0,
                            initGrasp
                            ]
        else:
            leftJoints = invArmKin(raven_constants.Arm.Left, initPose, initGrasp, True)
        
        self.prevLeftJoints = leftJoints
        self.prevRightJoints = None 
        self.prevPose = fwdArmKin(raven_constants.Arm.Left, leftJoints)[0] #FIXME: look at this
        self.indices = np.r_[self.manipIndices, [7, 8, 9]] # extend indices to match the actual dimension of the joints in the kinematics object
        self.publishORCommand(leftJoints, self.indices)
        self.robot.SetTransform(ROBOT_TRANSFORM)
        
        # configure laser scanner for sensing
        self.sensor = self.env.GetSensors()[0]
        self.sensor.Configure(rave.Sensor.ConfigureCommand.PowerOn)
        self.sensor.Configure(rave.Sensor.ConfigureCommand.RenderDataOn)
        
        #self.env.SetViewer('qtcoin')

    def publishORCommand(self, joints, joints_array_indices):
        """ Publish a command to the robot; sets joints directly"""
        if joints != None and joints_array_indices != None:
            self.robot.SetDOFValues(joints, joints_array_indices)

    def getORCommand(self, p, c, grip):
        """ Calculates joints based off of input from the leap motion """
        joints, joints_array_indices = self.calculateNewPose(p,c,grip)
        return joints, joints_array_indices


    def getExpectedMeasurement(self, pose, grasp=0, arm=raven_constants.Arm.Left, disp=False):
        """ Transforms the simulated Raven by the expected pose and raycasts to get the expected measurement"""
        success, joints, joints_array_indices = self.calculateJoints(pose, arm, grasp)
        if success:
            self.publishORCommand(joints, self.indices)
            start = time.clock()
            depth_im = self.getSensorData(disp=disp)
            stop = time.clock()
            print "Simulator query time (sec):", stop-start
        return depth_im
    
    def getSensorData(self, disp = False):
        start = time.clock()
        
        data = self.sensor.GetSensorData(rave.Sensor.Type.Laser)
        """
        while True:
            data = self.sensor.GetSensorData(self.sensor.Type.Laser)
            if data.stamp != olddata.stamp:
                break
            time.sleep(0.05)
        """
        # get points from data buffer    
        points = data.ranges
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
                depth_im[p[1], p[0]] = depths[i]
        
        if disp:
            plt.imshow(depth_im)
            plt.show()
        return depth_im

    #====================== OPEN RAVE HELPER FUNCTIONS =============================#
    def calculateJoints(self, pose, arm, grasp=0.0):
        #pose = ROBOT_TRANSFORM.dot(pose)
        try:
            result_joints_dict = invArmKin(arm, pose, grasp, True)
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