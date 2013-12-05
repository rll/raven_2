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
    def __init__(self):
        self.prob_occulusion = 0
        self.a = 0
        self.b = 0
        self.o = 0
        self.z = 0

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
    return STD_M*np.random.rand()+a

def prob_zg_b(pixel,z):
    prob = 0; 
    
    if z>0 and z< MAX_CAM_DEPTH:
        prob = BETA/MAX_CAM_DEPTH; 
        
    prob = prob + (1-BETA)*(STD_C*np.random.rand()+z)
    
    return prob


def prob_zpg_r_o(pixel,r):
    
    a = 0
    b = 0
    integral = 0
    distance = dist_to_object(r)
    while(a < distance):
        while(b<a):
            integral = integral + prob_z_b(pixel.z)*prob_a(a)*prob_b(pixel,a,b)
    
    return integral

        
def prob_ztg_r1tm1_o1tm1(pixels,r):
    prob = 1
    
    for pixel in pixels:
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
        rospy.Subscriber('raven_command',RavenCommand,self._stateCallback)
        rospy.Subscriber('Depth_Message',PoseStamped,_update)
        self.tracked_pose_pub = rospy.Publisher('Raven_Tracked_Pose', PoseStamped) 
        
        for i in range(0,NUM_OF_PARTICLES):
            particles.append(poses)
            
    def _updateRavenCommand(self,msg):
        
        for arm in msg.arms:
            if arm.name == self.arm:
                self.controlPose = self.controlPose*tfx.pose(arm.tool.pose, header=msg.header)
       
    def sample_from_r(self,particle):
        return (self.controlPose + STD_POSE*np.random.rand(4,4))*particle.pose
    
    def _update(self,msg):
        z = msg.z
        for particle in self.particles:
            current_pose = sample_from_r(self,particle)
            particle.likelihood = prob_ztg_r1tm1_o1tm1(pixels,current_pose)
            particle.current_pose = current_pose
            for pixel in pixels:
                pixel.prob_occulusion = update_prob_occulusion(pixel)
        
        for particle in self.particles:
            particle.pose = particle.likelihood*sample_from_r(self.particle)
        
        
        self.controlPose = np.identity(4)
        
        
            
##############################
#      EXECUTION CODE        #
##############################
def main():
    rospy.init_node('particle_filter',anonymous=True)
    