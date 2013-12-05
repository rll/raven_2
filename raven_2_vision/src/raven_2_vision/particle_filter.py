import roslib
import argparse
roslib.load_manifest('raven_2_vision')
import rospy


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

import message_filters
from threading import Lock

import tf.transformations as tft
import tfx

import IPython

from collections import namedtuple

########################################
#             CONSTANTS                #
########################################

LMBDA = 1
BETA = 0.01

#TODO: Check these Camera Specs 
MAX_CAM_DEPTH = 0.5 
STD_C = 0.01
STD_M = 0.01 
DT = 0.001

STD_POSE = 0.01 

P_NO_NO = 0.9
P_NO_O = 0.1
P_O_NO = 1-P_NO_NO
P_O_O = 1 - P_NO_O

NUM_OF_PARTICLES = 200

########################################
#             SUB-CLASSES               #
########################################
class Pixel(object):
    def __init__(self,x,y):
        self.prob_occulusion = 0
        self.x = x; 
        self.y = y; 
        self.a = 0
        self.b = 0
        self.o = 0
        self.z = 0
        self.depth = 0

class Particle(object):
    def __init__(self):
        self.current_pose
        self.likelihood = 0

########################################
#       Probability Distributions      #
########################################

def update_prob_occulusion(pixel):
    """
    Calculates p(o^i_t = 0|r_{1:t},z_{1:t},u_{1:t}) 
    """
    #O_{t-1} = 1
    PO_otm1_1 = update_prob_obser(pixel,0)*P_NO_O*(1-pixel.prob_occulusion)
    P0_otm1_0 = update_prob_obser(pixel,0)*P_NO_NO*(pixel.prob_occulusion)
     
    P1_otm1_0 = update_prob_obser(pixel,1)*P_O_NO*(pixel.prob_occulusion)
    P1_otm1_1 = update_prob_obser(pixel,1)*P_O_O*(1-pixel.prob_occulusion)
    
    prob = (PO_otm1_1+PO_otm1_O)/(PO_otm1_1+PO_otm1_O+P1_otm1_0+P1_otm1_1)
    pixel.prob_occulusion = prob
    

def gaussian(x,mean,std):
    return (1/(std*np.sqrt(2*np.pi))*np.exp(-(x-mean)^2/(2*std^2)))
    
def prob_b(pixel,a,b):
    
    if pixel.o == 0:
        if b - a == 0:
            return 1; 
        else:
            return 0; 
    else:
        if b<0 or b>a:
            return 0; 
        else:
            return LMBDA*np.exp(-LMBDA*b)/(1-np.exp(-LMBDA*a))

def prob_a(a):
    #TODO: Figure out distance from ray models
    return gaussian(a,mean,std)

def prob_zg_b(pixel,z,b):
    prob = 0; 
    
    if z>0 and z< MAX_CAM_DEPTH:
        prob = BETA/MAX_CAM_DEPTH; 
        
    prob = prob + (1-BETA)*(gaussian(z,b,STD_C))
    
    return prob


def prob_zpg_r_o(pixel,r):
    
    a = distance - STD_M
    b = 0
    integral = 0
    distance = pixel.depth
    while(a < distance+STD_M):
        while(b<a):
            integral = integral + prob_zg_b(pixel,z,b)*prob_a(pixel,a)*prob_b(pixel,a,b)
    
    return integral

        
def prob_ztg_r1tm1_o1tm1(pixels,r,depth_im):
    prob = 1
    
    for pixel in pixels:
        pixel.depth = depth_im(pixel)
        pixel.o = 0
        PO_otm1_1 = prob_xp_r_o(pixel,r)*P_NO_O*(1-pixel.prob_occulusion)
        P0_otm1_0 = prob_xp_r_o(pixel,r)*P_NO_NO*(pixel.prob_occulusion)
        
        pixel.o = 1
        P1_otm1_0 = prob_xp_r_o(pixel,r)*P_O_NO*(pixel.prob_occulusion)
        P1_otm1_1 = prob_xp_r_o(pixel,r)*P_O_O*(1-pixel.prob_occulusion)
        prob = prob*(PO_otm1_1+P0_otm1_0+P1_otm1_0+P1_otm1_1)
    
    return prob

class Particle_Filter:
    def __init__(self):
        self.particles = deque()
        rospy.Subscriber('raven_command',RavenCommand,self._updateRavenCommand)
        rospy.Subscriber('Depth_Message',PoseStamped,_update)
        self.tracked_pose_pub = rospy.Publisher('Raven_Tracked_Pose', PoseStamped) 
        self.RSimulator = RavenSimulator()
        self.StartPose = StartPose()
        
        
        for i in range(0,NUM_OF_PARTICLES):
            particle.pose = sample_from_r(self,self.StartPose)
            particle.pixels = deque()
            for y in range(1,YDIM_IN_IMAGE):
                for x in range(1,XDIM_IN_IMAGE):
                    p = pixel(x,y)
                    particle.pixels.append(p)
            self.particles.append(particle)
            
                    
                
            
    def _updateRavenCommand(self,msg):     
        for arm in msg.arms:
            if arm.name == self.arm:
                pose = tfx.pose(arm.position,arm.orientation,header=msg.header)
                self.controlPose = pose.matrix()*linalg.inv(self.prevpose.matrix())*self.controlPose
                self.prevpose = pose
       
    def sample_from_r(self,pose):
        return (self.controlPose + STD_POSE*np.random.rand(4,4))*particle.pose
    
    def _update(self,msg):
        z = msg.z
        norm = 0; 
        ml = 0; 
        for particle in self.particles:
            current_pose = sample_from_r(self,particle.pose)
            depth_im = self.Rsimulator(current_pose)
            particle.likelihood = prob_ztg_r1tm1_o1tm1(particle.pixels,current_pose,depth_im)
            norm += particle.likelihood
            
            if(particle.likelihood > ml):
                pubpose = particle.pose
                ml = particle.likelihood
                
            particle.current_pose = current_pose
            for pixel in particle.pixels:
                pixel.prob_occulusion = update_prob_occulusion(pixel)
        
        sample = np.array(norm*10)
        index = 0 
        for particle in self.particles:
            for i in range(index,index+floor(particle.likelihood*10)):
                sample[i] = particle
            
            index += floor(particle.likelihood*10)
        for particle in self.particles:
            particle.pose = sample(np.random.uniform(0,norm*10))
            
        
        self.tracked_pose_pub(pubpose)
        self.controlPose = np.identity(4)
        
        
            
##############################
#      EXECUTION CODE        #
##############################
def main():
    print("hi")
    rospy.init_node('particle_filter',anonymous=True)
    
    rospy.spin()
    