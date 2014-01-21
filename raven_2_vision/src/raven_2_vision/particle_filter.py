import roslib
import argparse
roslib.load_manifest('raven_2_vision')
import rospy

import cv 
import cv2
import cv_bridge
import numpy as np
from scipy import linalg
import math
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, Quaternion, PoseStamped, Polygon, PolygonStamped, Point32
import tf
import image_geometry
import time
import functools
from raven_or_simulator import RavenSimulator 
from raven_2_control.kinematics import *

import message_filters
from threading import Lock

import tf.transformations as tft
import tfx
from raven_2_msgs.msg import * 
import IPython
import copy

import matplotlib as mpl
import matplotlib.pyplot as plt

import time

from collections import namedtuple
from collections import deque

########################################
#             CONSTANTS                #
########################################

LMBDA = 1
BETA = 0.01

#TODO: Check these Camera Specs 
MAX_CAM_DEPTH = 0.5 
STD_C = 0.01
STD_M = 0.01
DT = 0.01
YDIM_IN_IMAGE = 240
XDIM_IN_IMAGE = 320
NUM_TOP_PARTICLES = 5 

DOWNSAMPLE = 1
YDIM_DOWN = YDIM_IN_IMAGE / DOWNSAMPLE
XDIM_DOWN = XDIM_IN_IMAGE / DOWNSAMPLE

STD_POSE = 0.01
SKIP_RATE = 2

PROB_MULTIPLIER = 1e10
MAGIC_CONSTANT = 76722 # email jmahler@berkeley.edu for questions

P_NO_NO = 0.9
P_NO_O = 0.1
P_O_NO = 1-P_NO_NO
P_O_O = 1 - P_NO_O

NUM_OF_PARTICLES = 100

HOME_POSE = tfx.pose([-.03,-.02,-.135],tfx.tb_angles(-90,90,0))
HOME_GRASP = 1.5


def downsampled_from_fullsize(i):
    return i/DOWNSAMPLE

def fullsize_from_downsampled(i):
    return i*DOWNSAMPLE

########################################
#             SUB-CLASSES               #
########################################

class Particle(object):
    def __init__(self):
        self.pose = tfx.pose([0,0,0])
        self.pixels = deque()
        self.likelihood = 0

########################################
#       Probability Distributions      #
########################################

def update_prob_occulusion(pixel,pose,depth_msr,depth_exp):
    """
    Calculates p(o^i_t = 0|r_{1:t},z_{1:t},u_{1:t}) 
    """
    #O_{t-1} = 1
    PO_otm1_1 = prob_zpg_r_o(pose,depth_msr,depth_exp,0)*P_NO_O*(1-pixel)
    PO_otm1_0 = prob_zpg_r_o(pose,depth_msr,depth_exp,0)*P_NO_NO*(pixel)
     
    P1_otm1_0 = prob_zpg_r_o(pose,depth_msr,depth_exp,1)*P_O_NO*(pixel)
    P1_otm1_1 = prob_zpg_r_o(pose,depth_msr,depth_exp,1)*P_O_O*(1-pixel)
    
    return (PO_otm1_1+PO_otm1_0)/(PO_otm1_1+PO_otm1_0+P1_otm1_0+P1_otm1_1)
    

def gaussian(x,mean,std):
    """
    GOOD
    """
    return (1/(std*np.sqrt(2*np.pi))*np.exp(-(x-mean)**2/(2*std**2)))
    
def prob_b(a,b,o):
    """
    Calculates p(b^i| a^i,o^i) 
    GOOD
    """
    if o == 0:
        if np.abs(b - a) < DT:
            return 1 / (2*DT); 
        else:
            return 0; 
    else:
        if b<=0 or b>a:
            return 0; 
        else:
            return LMBDA*np.exp(-LMBDA*b)/(1-np.exp(-LMBDA*a))

def prob_a(a,distance):
    """
    Calculates p(a^i| r)
    GOOD
    """
    return gaussian(a,distance,STD_M)

def prob_zg_b(z,b):
    """
    Calculates p(z^i| b^i)
    GOOD
    """
    prob = 0; 
    
    if z >= 0 and z <= MAX_CAM_DEPTH:
        prob = BETA/MAX_CAM_DEPTH; 
        
    prob = prob + (1-BETA)*(gaussian(z,b,STD_C))
    return prob


def prob_zpg_r_o(r,depth_msr,depth_exp,o):
    """
    Calculates p(z^i| r, o^i) 
    """
    b = 0
    integral_ab = 0
  
    a = depth_exp - 1*STD_M
    end_a = depth_exp + 1*STD_M
    
    while (a <= end_a):
        integral_b = 0
        b = a - DT
        while (b <= a):
            prob = prob_zg_b(depth_msr,b)*prob_a(a,depth_exp)*prob_b(a,b,o)
            integral_b += prob * DT
            b += DT
        integral_ab += integral_b * DT
        a += DT
        
    return integral_ab

        
def prob_ztg_r1tm1_o1tm1(pixels,r,depth_im,z):
    """
    Calculates p(z_t| r_{1:t},z_{1:t-1}) 
    """
    penalty = 0.1
    multiplier = PROB_MULTIPLIER
    prob = 0
    n_valid = 0
    for row in range(0,YDIM_DOWN):
        for col in range(0, XDIM_DOWN):
            
            row_f = fullsize_from_downsampled(row)
            col_f = fullsize_from_downsampled(col) 
            depth_exp = depth_im[row_f, col_f]
            depth_msr = z[row_f,col_f]
            i = row*XDIM_DOWN + col
            
            #print "DEPTHS", depth_exp, depth_msr
            
            if (depth_exp > 0 and depth_msr > 0 and depth_msr < MAX_CAM_DEPTH):
                o = 0
                PO_otm1_1 = prob_zpg_r_o(r,depth_msr,depth_exp,o)*P_NO_O*(1-pixels[i])
                P0_otm1_0 = prob_zpg_r_o(r,depth_msr,depth_exp,o)*P_NO_NO*(pixels[i])
            
                o = 1
                P1_otm1_0 = prob_zpg_r_o(r,depth_msr,depth_exp,o)*P_O_NO*(pixels[i])
                P1_otm1_1 = prob_zpg_r_o(r,depth_msr,depth_exp,o)*P_O_O*(1-pixels[i])
                
                prob_sum = np.max((PO_otm1_1+P0_otm1_0+P1_otm1_0+P1_otm1_1), 1.0 / multiplier)
                prob = prob + np.log(multiplier*prob_sum)
                # print "PROB", prob_sum
                n_valid += 1

    """
    if n_valid < 0.002*YDIM_DOWN*XDIM_DOWN:
        print "INVALID", n_valid
        prob = 0
    """
    #print "VALID", n_valid
    #print "PROB", prob
    return prob

class Particle_Filter:
    def __init__(self):
        self.particles = deque()
        self.arm = 'L'
        self.bridge = cv_bridge.CvBridge()
        
        self.grasp = HOME_GRASP
        self.controlPose = np.identity(4)
        self.myControlPose = self.controlPose
        self.lock = False
       
        self.StartPose = HOME_POSE
        self.StartGrasp = HOME_GRASP
        self.StartJoints = invArmKin(self.arm, self.StartPose, self.StartGrasp)
        self.prevpose = self.StartPose
        print 'Start'
        print self.prevpose.matrix
       
        self.RSimulator = RavenSimulator(initJoints=self.StartJoints)                    
        
        for i in range(0,NUM_OF_PARTICLES):
            particle = Particle()
            particle.pose = self.sample_from_r(self.StartPose, rot_noise=0.4, trans_noise=0.02)
            particle.pixels = np.zeros(YDIM_DOWN*XDIM_DOWN)

            self.particles.append(particle)

        self.depth_im_orig = self.RSimulator.getExpectedMeasurement(self.StartPose)
        self.numUpdates = 0
        self.publishedPose = False
        self.skipRate = SKIP_RATE  
        
        rospy.Subscriber('/bag/raven_command', RavenCommand, self._updateRavenCommand)
        rospy.Subscriber('/bag/downsampled_depth', Image, self._update)
         
        self.tracked_pose_pub = rospy.Publisher('raven_tracked_pose', PoseStamped) 
        
    def _updateRavenCommand(self,msg):     
        while(self.lock):
            i = 0
        
        self.lock = True
        for arm, arm_msg in zip(msg.arm_names, msg.arms):
            if arm == self.arm:
                
                pose = arm_msg.tool_command.pose
                pose = tfx.pose(pose.position,pose.orientation,header=msg.header)
                #if(np.sum(np.sum(np.abs((tfx.pose([0,0,0]).matrix - pose.matrix))))):
               # print "Pose"
                #print pose.matrix
               # print self.prevpose.matrix
                print "Delta", pose.matrix*linalg.inv(self.prevpose.matrix)
                self.controlPose = pose.matrix*linalg.inv(self.prevpose.matrix)*self.controlPose
                self.prevpose = copy.copy(pose)
                #print "got command"
        self.lock = False
       
    def sample_from_r(self,pose,rot_noise=0.1,trans_noise=0.002):
        rand_mat = np.eye(3,3) + rot_noise*(np.random.rand(3,3) - 0.5*np.ones((3,3)))
        U, S, V = np.linalg.svd(rand_mat)
        rand_rot = U.dot(V)
        rand_rot_pose = tfx.pose(rand_rot)
        new_pose = (tfx.random_translation_tf(scale=trans_noise,mean=0,dist='normal'))*self.myControlPose*pose
        new_pose = rand_rot_pose.matrix*new_pose
        return new_pose

    def getExpectedTrans(self,top_particles,total_likelihood):
        
        #Normailize likelihoods
        exp_trans = tfx.pose([0,0,0])
        for particle in top_particles:
            weight = particle.likelihood/totatl_likelihood
            exp_trans += weight* particle.pose.translation
            
        return exp_trans
            
    def _update(self, depth_msg):
        self.numUpdates = self.numUpdates + 1
        tot_likelihood = 0
        
        if self.numUpdates % self.skipRate != 0:
            return
        
        while(self.lock):
            i = 0
        # lock to wait for latest control pose information
        self.lock = True 
        self.myControlPose = copy.copy(self.controlPose)
        self.controlPose = np.identity(4)
        print "Control pose"
        print self.myControlPose
        self.lock = False
        
        # setup pf variables
        norm = 0
        ml = 0
        i = 0
        pointsPerImage = XDIM_DOWN * YDIM_DOWN
        z = self.bridge.imgmsg_to_cv(depth_msg, "32FC1")
        
        # profiling
        start = 0
        stop = 0
        start_whole_loop = time.clock()
        depth_im_best = []
        best_index = -1
        
        for particle in self.particles:
            print "PARTICLE:",i
            current_pose = self.sample_from_r(particle.pose)
            depth_im = self.RSimulator.getExpectedMeasurement(current_pose)
            #print current_pose
            
            if False:#self.numUpdates == 4:
                plt.figure()
                plt.subplot(111)
                plt.imshow(depth_im)
                plt.show()
            
            start = time.clock()
            particle.likelihood = 0
            if depth_im is not None:
                particle.likelihood = prob_ztg_r1tm1_o1tm1(particle.pixels, current_pose, depth_im, z)
                norm += particle.likelihood
            
            print "Likelihood", particle.likelihood
            # profile
            stop = time.clock()
            #print "Depth probability update time (sec)", stop - start
            start = time.clock()
            
            if(particle.likelihood >= ml):
                pubpose = tfx.pose(copy.copy(current_pose), stamp=rospy.Time.now(), frame='0_link')
                best_index = i
                print "Likelihood", particle.likelihood
                print "Best Pose", pubpose
                print "Best Index", best_index
                
                depth_im_best = copy.copy(depth_im)
                ml = particle.likelihood
            i += 1
            
            particle.pose = copy.copy(current_pose)
            if depth_im is not None:
                for row in range(0,YDIM_DOWN):
                    for col in range(0,XDIM_DOWN):
                        row_f = fullsize_from_downsampled(row)
                        col_f = fullsize_from_downsampled(col) 
                        depth_exp = depth_im[row_f,col_f]
                        depth_msr = z[row_f,col_f]
                        index = row*XDIM_DOWN + col
                        
                        if depth_exp <= 0 or depth_msr <= 0:
                            particle.pixels[index] = 0.5
                        else:
                            particle.pixels[index] = update_prob_occulusion(particle.pixels[index], current_pose, depth_msr, depth_exp)
            # profile
            stop = time.clock()
            #print "Occlusion probability update time (sec)", stop - start
            start = time.clock()
            
        print "NORM", norm
        index = 0
        bounds = [0]
        indices = [-1]
        norm_exp = 0
        
        for particle in self.particles:
            log_likelihood = particle.likelihood
            proportional_likelihood = np.exp((MAGIC_CONSTANT-pointsPerImage)*np.log(PROB_MULTIPLIER) + log_likelihood)
            norm_exp += proportional_likelihood
            bounds.append(bounds[-1] + proportional_likelihood)
            indices.append(indices[-1] + 1)
        bounds = bounds[1:-1] 
        
        d = zip(indices,bounds)
        new_particles = deque()
        
        for i in range(NUM_OF_PARTICLES):
            if norm_exp != 0:
                r = np.random.uniform(0,norm_exp)
                for b in range(len(bounds)):
                    if bounds[b] >= r:
                       # print "BOUNDS", b, i
                        new_particles.append(copy.copy(self.particles[b]))
                        break
            else:
                r = np.random.uniform(0,NUM_OF_PARTICLES-1)
                new_particles.append(copy.copy(self.particles[int(np.floor(r))]))
                
        self.particles = new_particles
        print "Best Index", best_index
        
        # profile
        stop = time.clock()
        print "Particle resampling time (sec)", stop - start
        start = time.clock()
        
        stop_whole_loop = time.clock()
        print "Whole loop took (sec)", stop_whole_loop - start_whole_loop
        
        if self.numUpdates <= 2:
            plt.figure()
            plt.subplot(141)
            plt.imshow(self.depth_im_orig)
            plt.subplot(142)
            plt.imshow(z)
            plt.subplot(143)
            plt.imshow(depth_im_best)
            plt.subplot(144)
            plt.imshow(depth_im_best, alpha=0.5)
            plt.imshow(z, alpha=0.5)
            plt.show()
        
        pose_msg = pubpose.msg.PoseStamped()
        pose_msg.header.frame_id = '0_link'
        
        self.tracked_pose_pub.publish(pose_msg)
        self.publishedPose = True
        
            
##############################
#      EXECUTION CODE        #
##############################
def main():
    print("hi")
<<<<<<< Updated upstream
    #IPython.embed()
=======
>>>>>>> Stashed changes
    rospy.init_node('particle_filter',anonymous=True)
    p = Particle_Filter()
    
    rospy.spin()
    
if __name__ == '__main__':
    main()
    