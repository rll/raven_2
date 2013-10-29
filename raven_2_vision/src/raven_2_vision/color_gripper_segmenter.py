#!/usr/bin/env python

# Import required Python code.
import roslib
import argparse
roslib.load_manifest('raven_2_vision')
import rospy

import cv
import cv2
import numpy as np
from scipy import linalg
import math
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
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

ColorDescription = namedtuple('ColorDescription','name lower upper')

ROI_size = namedtuple('ROI_size','width height')



########################################
#             CONSTANTS                #
########################################

# right gripper
GREEN_LOWER = (60, 60, 85) #THESE ARE THE GREEN PAPER VALUES
GREEN_UPPER = (105, 255, 255)
ORANGE_LOWER = (5, 130, 155) # THESE ARE THE ORANGE PAPER VALUES
ORANGE_UPPER = (35, 255, 255)

# left gripper
PURPLE_LOWER = (120, 65, 105)
PURPLE_UPPER = (170, 255, 255)
BLUE_LOWER = (105, 105, 90)
BLUE_UPPER = (135, 255, 255)

# TEMP: since only for one gripper at a time
#GREEN_LOWER = PURPLE_LOWER
#GREEN_UPPER = PURPLE_UPPER
#ORANGE_LOWER = BLUE_LOWER
#ORANGE_UPPER = BLUE_UPPER

DEFAULT_ROI_SIZE = ROI_size(300,200)

RED_LOWER = (0, 100, 30) # VALUES FOR THE RED FOAM
RED_UPPER = (3, 255, 255)

DEFAULT_RED_ROI_SIZE = ROI_size(200,100)

SLERP = True
DYNAMIC_SLERP = False
SLERP_CONSTANT = 0.1




########################################
#     3D GEOMETRY HELPER METHODS       #
########################################


def calculateSLERPConstant(angle):
    return math.fabs(angle/360)

def angleBetweenQuaternions(quat0, quat1):
    """ 
    Returns angle between quat0 and quat1 in degrees 
    """
    q0 = np.array(quat0)
    q1 = np.array(quat1)
    theta = math.acos(2*np.dot(q0,q1)**2 - 1)
    theta = theta*(180.0/math.pi)
    return theta



def order_points(p1, p2):
    """ 
    Takes in two points of form (x,y) and orders them according to y-coordinate value 
    """
    if p1[1] > p2[1]:
        return p1,p2
    elif p1[1] < p2[1]:
        return p2,p1
    else:
        return p1,p2



def find_endpoints(contours):
    """ 
    Takes in a list of contours and returns the pixel coordinates of centroids of the top two contours 
    """
    if len(contours) < 2:
        return (0,0,False)
    contours.sort(key=len, reverse=True)
    points = []
    for c in contours:
        moments = cv2.moments(c)
        if moments['m00']!=0:
            cx = int(moments['m10']/moments['m00'])     
            cy = int(moments['m01']/moments['m00'])        
            points.append((cx, cy))
    try:
        upper_pt, lower_pt = order_points(points[0], points[1])
        return (lower_pt, upper_pt, True)
    except IndexError as e:
        return (0,0,False)        
    else:
        return (0,0,False)


def quat_dot_product(q1, q2):
    """
    takes in two quaternions and returns their dot product
    """
    return q1.x*q2.x + q1.y*q2.y + q1.z*q2.z +q1.w*q2.w


def slerp(q1, q2, t):
    """
    finds the SLERP interpolation between two quaternions
    """
    dot_product = quat_dot_product(q1,q2)
    if dot_product < 0:
        q1.w = -q1.w
        dot_product = quat_dot_product(q1, q2)
    theta = math.acos(dot_product)
    x = (math.sin((1-t)*theta)/math.sin(theta))*q1.x + (math.sin(t*theta)/math.sin(theta))*q2.x
    y = (math.sin((1-t)*theta)/math.sin(theta))*q1.y + (math.sin(t*theta)/math.sin(theta))*q2.y
    z = (math.sin((1-t)*theta)/math.sin(theta))*q1.z + (math.sin(t*theta)/math.sin(theta))*q2.z
    w = (math.sin((1-t)*theta)/math.sin(theta))*q1.w + (math.sin(t*theta)/math.sin(theta))*q2.w
    result = Quaternion()
    result.x = x
    result.y = y
    result.z = z
    result.w = w
    return result


########################################
#    IMAGE-RELATED HELPER METHODS      #
########################################

def threshold(image, hsvImg, threshImg, lowerHSV, upperHSV, erode_and_dilate=True):
    """ 
    Thresholds an image for a certain range of hsv values 
    """ 
    cv.Smooth(image, image, cv.CV_GAUSSIAN, 3, 0)
    cv.CvtColor(image, hsvImg, cv.CV_BGR2HSV) 
    cv.InRangeS(hsvImg, lowerHSV, upperHSV, threshImg)
    if erode_and_dilate:
        cv.Erode(threshImg, threshImg, None, 2)
        cv.Dilate(threshImg, threshImg, None, 2)
        #cv.Erode(threshImg, threshImg, None, 2)    
        #cv.Dilate(threshImg, threshImg, None, 1)
        #cv.Erode(threshImg, threshImg, None, 1)
        cv.Dilate(threshImg, threshImg, None, 1)
        cv.Erode(threshImg, threshImg, None, 1)
    return threshImg

def average_vectors(v1, v2):
    """
    Finds the average of two vectors v1 and v2
    """
    result = []
    for i in range(0, len(v1)):
        result.append((v1[i]+v2[i])/2)
    return result

def find_contours(im, name):
    """ 
    Takes in a thresholded image and returns a list of the contours 
    """
    thresh = im
    im = np.asarray(im[:,:])
    contours, hierarchy = cv2.findContours(np.asarray(thresh[:,:]),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours

class ColorSidePoints(object):
    def __init__(self):
        self.found = False
        self.upper = None
        self.lower = None

class ColorPoints(object):
    def __init__(self, name, hsv_lower, hsv_upper):
        self.name = name
        self.hsv_lower = cv.Scalar(*hsv_lower)
        self.hsv_upper = cv.Scalar(*hsv_upper)
        self.left = ColorSidePoints()
        self.right = ColorSidePoints()
    
    def __getitem__(self, name):
        return getattr(self, name)
    
#########################
# PUBLISHING METHODS    #
#########################

def createMarker(pose, color="red", id=None, lifetime=None):
    id = id or 0
    lifetime = lifetime or 2
    marker = Marker()
    marker.id = id
    marker.header.frame_id = pose.header.frame_id
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.002
    marker.scale.y = 0.002
    marker.scale.z = 0.002
    marker.color.a = 1.0
    if color == "red":
        marker.color.r = 255
        marker.color.b = 0
    else:
        marker.color.r = 0
        marker.color.b = 255
    marker.color.g = 0
    marker.pose = pose.pose
    marker.lifetime = rospy.Duration(lifetime)
    return marker

##################
#   MAIN CLASS   #
##################

class ColorSegmenter(object):
    def __init__(self, arm, camera_name, gripper_ROI_wh, color1=None,color2=None, **kwargs):
        self.arm = arm
        self.sides = ['left','right']
        self.camera_name = camera_name
        self.camera = dict((side, '%s/%s' % (camera_name,side)) for side in self.sides)
        
        self.show_images = kwargs.get('show_images')
        self.show_thresh_red_images = kwargs.get('show_thresh_red_images')
        self.print_messages = kwargs.get('print_messages')
        self.show_time = kwargs.get('show_time')
        
        self.tool_frame = '/tool_' + self.arm
        self.camera_frame = None
        
        self.color1 = ColorPoints(*color1)
        self.color2 = ColorPoints(*color2)
        
        
        self.locks = dict((side,Lock()) for side in self.sides)
        self.listener = tfx.TransformListener()
        self.bridge = cv_bridge.CvBridge()
        
        self.info = {}
        self.img = {}
        
        self.gripper_ROI_size = ROI_size(*gripper_ROI_wh)
        self.gripper_ROI = {}
        
        self.red = kwargs['red']
        self.red_ROI_size = kwargs['red_ROI_size']
        self.red_ROI = {}
        
        self.t0 = {}
        
        self.offset_from_tool_base = .005
        
        self.prevQuaternion = None
        
        self.detected_gripper_pose = None
        self.estimated_gripper_pose = None

        self.quat_pub = rospy.Publisher('tape_orientation_' + self.arm, Quaternion)
        self.pose_pub = rospy.Publisher('tape_pose_' + self.arm, PoseStamped)
        self.polygon_pub = rospy.Publisher('polygon_' + self.arm, PolygonStamped)
        self.polygon_pub_base = rospy.Publisher('polygon_base_'+self.arm, PolygonStamped)
        self.gripper_ROI_pub = dict((side,rospy.Publisher('%s_ROI_%s' % (self.arm, side), Image)) for side in self.sides) 
        self.color1_pub = dict((side,rospy.Publisher('%s_%s_%s' % (self.arm, self.color1.name, side), Image)) for side in self.sides)
        self.color2_pub = dict((side,rospy.Publisher('%s_%s_%s' % (self.arm, self.color2.name, side), Image)) for side in self.sides)
        
        rospy.Subscriber('estimated_gripper_pose_%s' % self.arm, PoseStamped, self._estimatedPoseCallback)

        for side in self.sides:
            rospy.Subscriber('%s/camera_info' % self.camera[side], CameraInfo, functools.partial(self._infoCallback,side))
            rospy.Subscriber('%s/image_rect_color' % self.camera[side], Image, functools.partial(self._imageCallback,side))
        
        self.red_thresh_pub = rospy.Publisher('found_red_%s'%self.arm, Bool)
        
        self.lastFoundRed = None
        self.redHistoryDuration = rospy.Duration(.5)
        
        
        self.red_roi_pub = rospy.Publisher('red_roi_%s'%self.arm, Image)
        self.red_thresh_roi_pub = rospy.Publisher('red_thresh_roi_%s'%self.arm, Image)
        
        self.red_thresh_marker_pub = rospy.Publisher("found_red_marker_%s"%self.arm, Marker)
    
    ################################
    #   SUBSCRIBER BOUND METHODS   #
    ################################
    def _infoCallback(self, side, info):
        self.info[side] = info
        if side == 'left' and not self.camera_frame:
            self.camera_frame = info.header.frame_id
    
    def _imageCallback(self, side, image):
        """         
        Takes in the image from the camera and processes it 
        """
        if self.locks[side].locked_lock():
            return

        #with self.locks[side]:
        if True:
            for info in self.info.values():
                if not info:
                    return
            #if not self.info.has_key(side):
            #    return

            self.t0[side] = time.clock()
            color2, color1, color2_contour, color1_contour = self.process(image, side)
            img = self.gripper_ROI[side]
            self.gripper_ROI_pub[side].publish(self.bridge.cv_to_imgmsg(img))
            self.color1_pub[side].publish(self.bridge.cv_to_imgmsg(color1))
            self.color2_pub[side].publish(self.bridge.cv_to_imgmsg(color2))
            if self.show_images:
                cv.ShowImage("%s arm %s camera ROI" % (self.arm, side), img)
                cv.ShowImage("%s arm %s camera %s" % (self.arm, side, self.color2.name), color2)
                cv.ShowImage("%s arm %s camera %s" % (self.arm, side, self.color1.name), color1)
            cv.WaitKey(3)
            self.handleBoth(image.header.stamp)
            self.color2[side].found = self.color1[side].found = False
            
            self.threshRed()
            
    def _estimatedPoseCallback(self, msg):
        pose = tfx.pose(msg)
        if self.camera_frame is not None:
            tf_poseframe_to_cam = tfx.lookupTransform(self.camera_frame,pose.frame)
            self.estimated_gripper_pose = tf_poseframe_to_cam * pose

    def getEstimatedPose(self):
        if self.estimated_gripper_pose is not None:
            estimated_gripper_pose = self.estimated_gripper_pose
            estimated_time = estimated_gripper_pose.stamp.seconds
        else:
            pose = tfx.pose([0,0,0], [0,0,0,1], frame=self.tool_frame, stamp=rospy.Time.now())
            tf_tool_to_cam = tfx.lookupTransform(self.camera_frame,self.tool_frame)
            estimated_gripper_pose = tf_tool_to_cam * pose
            estimated_time = rospy.Time.now().secs
            
        #estimated_time = estimated_gripper_pose.stamp.seconds
        detected_time = self.detected_gripper_pose.stamp.seconds if self.detected_gripper_pose is not None else -float("inf")
        
        if not estimated_time > detected_time:
            if math.fabs(estimated_time - detected_time) < 1:
                time_since_detection = 0
            else:
                time_since_detection = 0#float("inf")
        else:
            time_since_detection = estimated_time - detected_time
        #time_since_detection = (estimated_time - detected_time) if estimated_time > detected_time else float("inf")
        
        if self.print_messages:
            if self.detected_gripper_pose is None:
                rospy.loginfo('detected_gripper_pose is None')
            rospy.loginfo('estimated_time - detected_time = {0}'.format(estimated_time - detected_time))
            
        self.estimated_gripper_pose = None
        return estimated_gripper_pose, time_since_detection

    def threshRed(self):
        THRESH_RED_WIDTH_ROI = 200
        THRESH_RED_HEIGHT_ROI = 90
        
        if not self.img.has_key('left') or not self.img.has_key('right'):
            return

        leftImg = cv.CloneMat(self.img['left'])
        rightImg = cv.CloneMat(self.img['right'])
        width = cv.GetSize(leftImg)[0]
        height = cv.GetSize(leftImg)[1]
        
        estimated_gripper_pose, time_since_detection = self.getEstimatedPose()

        gripper_pos = tfx.pose(estimated_gripper_pose).position
        tf_tool_to_cam = tfx.lookupTransform('/left_BC',gripper_pos.frame,wait=10)        
        gripper_pos = tf_tool_to_cam * gripper_pos
        stereoModel = image_geometry.StereoCameraModel()
        stereoModel.fromCameraInfo(self.info['left'], self.info['right'])
        (u_l, v_l), (u_r, v_r) = stereoModel.project3dToPixel(gripper_pos.list)
        
        def expandROIRegion(start_ROI, elapsed_time, max_elapsed_time=20., growth_factor=2.):
            elapsed_time = min(elapsed_time,max_elapsed_time)
            
            return int(start_ROI + start_ROI * (elapsed_time/max_elapsed_time) * growth_factor)
        
        #THRESH_RED_WIDTH_ROI = expandROIRegion(THRESH_RED_WIDTH_ROI, time_since_detection)
        #THRESH_RED_HEIGHT_ROI = expandROIRegion(THRESH_RED_HEIGHT_ROI, time_since_detection)

        leftImg_lb_width = int(u_l-THRESH_RED_WIDTH_ROI) if int(u_l-THRESH_RED_WIDTH_ROI) > 0 else 0
        leftImg_ub_width = int(u_l+THRESH_RED_WIDTH_ROI) if int(u_l+THRESH_RED_WIDTH_ROI) < width else width-1
        leftImg_lb_height = int(v_l-THRESH_RED_HEIGHT_ROI) if int(v_l-THRESH_RED_HEIGHT_ROI) > 0 else 0
        leftImg_ub_height = int(v_l+THRESH_RED_HEIGHT_ROI) if int(v_l+THRESH_RED_HEIGHT_ROI) < height else height-1
        rightImg_lb_width = int(u_r-THRESH_RED_WIDTH_ROI) if int(u_r-THRESH_RED_WIDTH_ROI) > 0 else 0
        rightImg_ub_width = int(u_r+THRESH_RED_WIDTH_ROI) if int(u_r+THRESH_RED_WIDTH_ROI) < width else width-1
        rightImg_lb_height = int(v_r-THRESH_RED_HEIGHT_ROI) if int(v_r-THRESH_RED_HEIGHT_ROI) > 0 else 0
        rightImg_ub_height = int(v_r+THRESH_RED_HEIGHT_ROI) if int(v_r+THRESH_RED_HEIGHT_ROI) < height else height-1

        if self.print_messages:
            print('(height, width) = ({0}, {1})'.format(height,width))
            print('time_since_detection: {0}'.format(time_since_detection))
            print('leftImg[{0}:{1},{2}:{3}]'.format(leftImg_lb_height,leftImg_ub_height,leftImg_lb_width,leftImg_ub_width))
            print('rightImg[{0}:{1},{2}:{3}]'.format(rightImg_lb_height,rightImg_ub_height,rightImg_lb_width,rightImg_ub_width))
        
        if leftImg_lb_width >= leftImg_ub_width or leftImg_lb_height >= leftImg_ub_height or rightImg_lb_width >= rightImg_ub_width or rightImg_lb_height >= rightImg_ub_height:
            return

        leftImg = leftImg[leftImg_lb_height:leftImg_ub_height, leftImg_lb_width:leftImg_ub_width]
        rightImg = rightImg[rightImg_lb_height:rightImg_ub_height, rightImg_lb_width:rightImg_ub_width]
        if self.show_thresh_red_images:
            self.red_roi_pub.publish(self.bridge.cv_to_imgmsg(leftImg))

        leftThresh = cv.CreateImage(cv.GetSize(leftImg),8,1)
        rightThresh = cv.CreateImage(cv.GetSize(rightImg),8,1)        
        leftThresh = threshold(leftImg, leftImg, leftThresh, RED_LOWER, RED_UPPER)
        rightThresh = threshold(rightImg, rightImg, rightThresh, RED_LOWER, RED_UPPER)
        
        if self.show_thresh_red_images:
            self.red_thresh_roi_pub.publish(self.bridge.cv_to_imgmsg(leftThresh))

        leftContours = find_contours(leftThresh, "leftThresh")
        rightContours = find_contours(rightThresh, "rightThresh")
        foundRed = True
        if len(leftContours) > 0 or len(rightContours) > 0:
            self.lastFoundRed = rospy.Time.now()
            foundRed = True
        else:
            if self.lastFoundRed is not None and rospy.Time.now() - self.lastFoundRed < self.redHistoryDuration:
                foundRed = True
            else:
                foundRed = False

        toolPose = tfx.pose([0,0,0],frame=self.tool_frame,stamp=rospy.Time.now()).msg.PoseStamped()
        if foundRed:
            self.red_thresh_marker_pub.publish(createMarker(toolPose,color="red"))
        else:
            self.red_thresh_marker_pub.publish(createMarker(toolPose,color="blue"))
        
        self.red_thresh_pub.publish(Bool(foundRed))


    def process(self, image, side):
        """
        thresholds the image for a certain hsv range and returns the coordinates of the centroid, 
        and the coordinates of the closest point to the centroid
        """
        cv_image = self.bridge.imgmsg_to_cv(image, "bgr8")
        self.img[side] = cv_image
        
        width = cv.GetSize(cv_image)[0]
        height = cv.GetSize(cv_image)[1]
        
        ROI, pixel = self.determineROI(side)
        
        estimated_gripper_pose, time_since_detection = self.getEstimatedPose()
        
        def expandROIRegion(start_ROI, elapsed_time, upper_bound, pixel_per_sec=.1):
            ROI_size =  start_ROI + start_ROI * (elapsed_time * pixel_per_sec)
            return int(min(ROI_size, upper_bound))
        
        ROI_height = expandROIRegion(self.gripper_ROI_size.height, time_since_detection, height)
        ROI_width = expandROIRegion(self.gripper_ROI_size.width, time_since_detection, width)
        
        
        lower_height_bound = int(pixel[1]) - ROI_height
        lower_width_bound = int(pixel[0]) - ROI_width
        upper_height_bound = lower_height_bound + 2*ROI_height
        upper_width_bound = lower_width_bound + 2*ROI_width
        
        lower_height_bound = lower_height_bound if lower_height_bound > 0 else 0
        lower_width_bound = lower_width_bound if lower_width_bound > 0 else 0
        upper_height_bound = upper_height_bound if upper_height_bound < height else height - 1
        upper_width_bound = upper_width_bound if upper_width_bound < width else width - 1
        
        
        if lower_height_bound >= upper_height_bound:
             lower_height_bound = 0
             upper_height_bound = height
        if lower_width_bound >= upper_width_bound:
            lower_width_bound = 0
            upper_width_bound = width
        
        
        if self.print_messages:
            print('Pixel (height, width): ({0},{1})'.format(pixel[1],pixel[0]))
            print('ROI (height, width): ({0}, {1})'.format(ROI_height, ROI_width))
            print('Process image bounds: [{0} : {1} , {2} : {3}]'.format(lower_height_bound,upper_height_bound,
                                                                    lower_width_bound, upper_width_bound))
        
        cv_image = cv_image[lower_height_bound:upper_height_bound, lower_width_bound:upper_width_bound]
        
        self.gripper_ROI[side] = cv_image
        
        hsvImg = cv.CreateImage(cv.GetSize(cv_image),8,3)
        color1ThreshImg = cv.CreateImage(cv.GetSize(cv_image),8,1)
        color1ThreshImg = threshold(cv_image, hsvImg, color1ThreshImg, self.color1.hsv_lower, self.color1.hsv_upper)
        color2ThreshImg = cv.CreateImage(cv.GetSize(cv_image),8,1)
        color2ThreshImg = threshold(cv_image, hsvImg, color2ThreshImg, self.color2.hsv_lower, self.color2.hsv_upper)
        color2ContourImg = cv.CloneImage(color2ThreshImg)
        color1ContourImg = cv.CloneImage(color1ThreshImg)
        color2_contours = find_contours(color2ContourImg, self.color2.name) #FIXME remove later
        color1_contours = find_contours(color1ContourImg, self.color1.name) #FIXME remove later
        self.color2[side].lower, self.color2[side].upper, self.color2[side].found = find_endpoints(color2_contours)
        self.color1[side].lower, self.color1[side].upper, self.color1[side].found = find_endpoints(color1_contours)
        self.color2[side].lower = self.addOffset(self.color2[side].lower, lower_width_bound, lower_height_bound)
        self.color2[side].upper = self.addOffset(self.color2[side].upper, lower_width_bound, lower_height_bound)
        self.color1[side].lower = self.addOffset(self.color1[side].lower, lower_width_bound, lower_height_bound)
        self.color1[side].upper = self.addOffset(self.color1[side].upper, lower_width_bound, lower_height_bound)
        return color2ThreshImg, color1ThreshImg, color2ContourImg, color1ContourImg


    def handleBoth(self, image_time=None):
        """ 
        Returns the quaternion and position of the pieces of tape (which should correspond to the 
        orientation and position of the gripper 
        """
        image_time = image_time or rospy.Time.now()
        
        try:
            found = self.color1.left.found and self.color2.left.found and self.color1.right.found and self.color2.right.found
            if self.print_messages:
                print "found"
                print "left_%s:\t\t%s" % (self.color1.name,self.color1.left.found)
                print "right_%s:\t\t%s" % (self.color1.name,self.color1.right.found)
                print "left_%s:\t\t%s" % (self.color2.name,self.color2.left.found)
                print "right_%s:\t\t%s" % (self.color2.name,self.color2.right.found)
            if found:
                    
                
                color2_lower_pt = self.convertStereo(self.color2.left.lower[0], self.color2.left.lower[1], math.fabs(self.color2.left.lower[0] - self.color2.right.lower[0]))
                color2_upper_pt = self.convertStereo(self.color2.left.upper[0], self.color2.left.upper[1], math.fabs(self.color2.left.upper[0] - self.color2.right.upper[0]))
                color1_lower_pt = self.convertStereo(self.color1.left.lower[0], self.color1.left.lower[1], math.fabs(self.color1.left.lower[0] - self.color1.right.lower[0]))
                color1_upper_pt = self.convertStereo(self.color1.left.upper[0], self.color1.left.upper[1], math.fabs(self.color1.left.upper[0] - self.color1.right.upper[0]))
                
                
                polygon_points = [color2_lower_pt, color2_upper_pt, color1_upper_pt, color1_lower_pt]
                polygon_points = [Point32(*x.tolist()) for x in polygon_points]
                polygon = PolygonStamped()
                polygon.polygon.points = polygon_points
                polygon.header.frame_id = "left_BC"
                polygon.header.stamp = rospy.Time.now()
                self.polygon_pub_base.publish(polygon)
                
                
                points = self.get_corrected_points_from_plane(color1_lower_pt, color1_upper_pt, color2_lower_pt, color2_upper_pt)
               
                color1_lower_pt = points[0]
                color1_upper_pt = points[1]
                color2_lower_pt = points[2]
                color2_upper_pt = points[3] 
                
                
                color2_vector = []
                color1_vector = []
                for i in range(0, 3):
                    color2_vector.append(color2_upper_pt[i] - color2_lower_pt[i])
                    color1_vector.append(color1_upper_pt[i] - color1_lower_pt[i])
                
                origin = (color1_lower_pt + color2_lower_pt)/2.
                p = tfx.point(self.get_position_from_lines(origin, color1_vector, color2_vector)).msg.Point()
                
                
                #p0 = Point()
                #position = (color2_lower_pt+color2_upper_pt+color1_lower_pt+color1_upper_pt)/4
                #p0.x = position[0]
                #p0.y = position[1]
                #p0.z = position[2]
                #print 'p0: {0}\n'.format(p0)

                polygon_points = [color2_lower_pt, color2_upper_pt, color1_upper_pt, color1_lower_pt]
                polygon_points = [Point32(*x.tolist()) for x in polygon_points]
                polygon = PolygonStamped()
                polygon.polygon.points = polygon_points
                polygon.header.frame_id = "left_BC"
                polygon.header.stamp = rospy.Time.now()
                self.polygon_pub.publish(polygon)
                
                quat = self.get_orientation_from_lines(color2_vector, color1_vector)
                tb = tfx.tb_angles(quat)
                q = Quaternion()
                q.x = quat[0]
                q.y = quat[1]
                q.z = quat[2]
                q.w = quat[3]

                if self.prevQuaternion == None:
                    self.prevQuaternion = q
                else:
                    prevQuat = tfx.tb_angles(self.prevQuaternion).msg
                    currQuat = tfx.tb_angles(q).msg
                    if SLERP:
                        try:
                            """if DYNAMIC_SLERP:
                                angle = angleBetweenQuaternions(prevQuat, currQuat)
                                print "ANGLE: "+angle
                                slerp_constant = calculateSLERPConstant(angle)
                            else:
                                slerp_constant = SLERP_CONSTANT
                            print "SLERP: "+slerp_constant"""
                            q = slerp(prevQuat, currQuat, SLERP_CONSTANT)
                        except ZeroDivisionError:
                            q = currQuat
                        self.prevQuaternion = q
                    else:
                        q = currQuat
                        self.prevQuaternion = q
                self.detected_gripper_pose = tfx.pose(p,q,frame=self.camera_frame,stamp=image_time)
                self.quat_pub.publish(q)
                self.pose_pub.publish(self.detected_gripper_pose.msg.PoseStamped())
                t_left = time.clock() - self.t0['left']
                t_right = time.clock() - self.t0['right']
                if self.show_time:
                    print "RUNNING TIME FROM LEFT IMAGE: "+str(t_left)
                    print "RUNNING TIME FROM RIGHT IMAGE: "+str(t_right)
                if self.print_messages:                
                    print "WORKING FINE"
            else:
                #self.pose_pub.publish(self.prevQuaternion) #if the points aren't detected, continue publishing the last known location
                if self.print_messages:                
                     print "POINTS NOT DETECTED"
        except (TypeError, ValueError) as e:
            if self.print_messages:
                print "CAUGHT ERROR"    
                print e
            #self.pose_pub.publish(self.prevQuaternion) #if some kind of error comes up, continue publishing the last known location #FIXME we may want to change this behavior
            pass



    ##############################
    #       HELPER METHODS       #
    ############################## 

    ########################################
    #       RED THRESHOLD PUBLISHER        #
    ########################################


    def determineROI(self, side=None):
        """
        Determines the boundaries of a region of interest, based on 
        the pixel coordinates of the gripper as given by the inverse kinematics of the robot
        """
        ROI_width = self.gripper_ROI_size.width
        ROI_height = self.gripper_ROI_size.height
        
        pose, timeSinceDetection = self.getEstimatedPose()
        stereoModel = image_geometry.StereoCameraModel()
        stereoModel.fromCameraInfo(self.info['left'], self.info['right'])
        (u_l, v_l), (u_r, v_r) = stereoModel.project3dToPixel(pose.position.list)
        ROI_left = ((int(u_l-ROI_width), int(v_l-ROI_height)), (int(u_l+ROI_width), int(v_l+ROI_height)))
        ROI_right= ((int(u_r-ROI_width), int(v_r-ROI_height)), (int(u_r+ROI_width), int(v_r+ROI_height)))
        ROI = {'left': ROI_left, 'right': ROI_right}
        uv = {'left': (u_l, v_l), 'right': (u_r, v_r)}
        if side is None:
            return ROI, uv
        else:
            return ROI[side], uv[side]
        

    def addOffset(self, pt, xOffset, yOffset):
        """
        Adds x and y offsets to a point of form (x, y)
        """
        if type(pt) == tuple:
            return (pt[0]+xOffset,pt[1]+yOffset)
        return pt

    def get_position_from_lines(self, origin, v0, v1):
        origin, v0, v1 = np.array(origin), np.array(v0), np.array(v1)
        offset_vector = v0 + v1
        offset_vector = self.offset_from_tool_base*(offset_vector/np.linalg.norm(offset_vector))
        return origin + offset_vector
    
    def get_corrected_points_from_plane(self,p1,p2,p3,p4):
        """
        Uses least squares to compute a plane between all points then recalculates z 
        component to correct for error in stereo
        """
        
        A = np.array([p1,p2,p3,p4])
        B = np.array([[1],[1],[1],[1]])
       
        plane = np.linalg.lstsq(A,B)
        
        p1[2] = (1-plane[0][0,0]*p1[0]-plane[0][1,0]*p1[1])/plane[0][2,0]  
        p2[2] = (1-plane[0][0,0]*p2[0]-plane[0][1,0]*p2[1])/plane[0][2,0] 
        p3[2] = (1-plane[0][0,0]*p3[0]-plane[0][1,0]*p3[1])/plane[0][2,0] 
        p4[2] = (1-plane[0][0,0]*p4[0]-plane[0][1,0]*p4[1])/plane[0][2,0]  
         
        return np.array([p1,p2,p3,p4])

    def get_orientation_from_lines(self, v0, v1):
        """ 
        Takes in two vectors, v0 and v1, (which are KNOWN to be in the same plane) and finds 
        the normal, and creates a rotation matrix from v0, v1, and the normal; then converts this 
        rotation matrix to a quaternion 
        """
        v0, v1 = np.array(v0), np.array(v1)
        v0 = v0 / np.linalg.norm(v0) 
        v1 = v1 / np.linalg.norm(v1) 
        n = np.cross(v0, v1)
        parallel = average_vectors(v0, v1)
        parallel = parallel / np.linalg.norm(parallel)
        third = np.cross(n, parallel)
        third = third / np.linalg.norm(third)
        #n = n / np.linalg.norm(n)
        #v1 = np.cross(n, v0)
        #v1 = v1 / np.linalg.norm(v1)
        #rotMat = np.vstack((n, v1, v0)).T
        rotMat = np.vstack((parallel, third, n)).T
        matrix = rotMat
        tbRot = tfx.tb_angles(matrix).matrix        
        #tbRot = self.rotate(-90, "yaw", tbRot)    #FIXME: get correct yaw pitch roll values
        #tbRot = self.rotate(60, "pitch", tbRot)
        tbRot = self.rotate(180, "roll", tbRot)
        quat = tfx.tb_angles(tbRot).quaternion
        return list(quat)


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


    def convertStereo(self, u, v, disparity):
        """ 
        Converts two pixel coordinates u and v along with the disparity to give PointStamped 
        """
        stereoModel = image_geometry.StereoCameraModel()
        stereoModel.fromCameraInfo(self.info['left'], self.info['right'])
        (x,y,z) = stereoModel.projectPixelTo3d((u,v), disparity)
        return np.array((x,y,z))


##############################
#      EXECUTION CODE        #
##############################

def main():
    rospy.init_node('color_segmenter',anonymous=True)
    
    parser = argparse.ArgumentParser()
    
    parser.add_argument('arm',nargs='?')
    parser.add_argument('--camera-name')
    parser.add_argument('--show-images','-i',action='store_true',default=False)
    parser.add_argument('--show-thresh-red-images',action='store_true',default=False)
    parser.add_argument('--print-messages',action='store_true',default=False)
    parser.add_argument('--show-time',action='store_true',default=False)
    
    parser.add_argument('--roi',nargs=2,type=int, dest='ROI_size', metavar=('width','height'))
    
    color1_group = parser.add_argument_group('color1','color1 options')
    color1_group.add_argument('--color1-name')
    color1_group.add_argument('--color1-hsv',nargs=6,type=int,metavar=('lower_h','lower_s','lower_v','upper_h','upper_s','upper_v'))
    color1_group.add_argument('--color1-lower',nargs=3,type=int,metavar=('h','s','v'))
    color1_group.add_argument('--color1-upper',nargs=3,type=int,metavar=('h','s','v'))
    
    
    color2_group = parser.add_argument_group('color2','color2 options')
    color2_group.add_argument('--color2-name')
    color2_group.add_argument('--color2-hsv',nargs=6,type=int,metavar=('lower_h','lower_s','lower_v','upper_h','upper_s','upper_v'))
    color2_group.add_argument('--color2-lower',nargs=3,type=int,metavar=('h','s','v'))
    color2_group.add_argument('--color2-upper',nargs=3,type=int,metavar=('h','s','v'))
    
    red_group = parser.add_argument_group('red','red options')
    red_group.add_argument('--red-roi',nargs=2,type=int,dest='red_ROI_size',metavar=('width','height'))
    red_group.add_argument('--red-hsv',nargs=6,type=int,metavar=('lower_h','lower_s','lower_v','upper_h','upper_s','upper_v'))
    red_group.add_argument('--red-lower',nargs=3,type=int,metavar=('h','s','v'))
    red_group.add_argument('--red-upper',nargs=3,type=int,metavar=('h','s','v'))
    
    args = parser.parse_args(rospy.myargv()[1:])

    arm = args.arm or rospy.get_param('~arm','R')
    del args.arm
    
    camera_name = args.camera_name or '/BC'
    del args.camera_name    

    if arm == 'R':
        color2_name = 'orange'
        color2_lower = ORANGE_LOWER
        color2_upper = ORANGE_UPPER
        color1_name = 'green'
        color1_lower = GREEN_LOWER
        color1_upper = GREEN_UPPER
    else:
        color1_name = 'blue'
        color1_lower = BLUE_LOWER
        color1_upper = BLUE_UPPER
        color2_name = 'purple'
        color2_lower = PURPLE_LOWER
        color2_upper = PURPLE_UPPER
    
    color1_name = args.color1_name or color1_name
    color2_name = args.color2_name or color2_name
    
    if args.color1_hsv:
        color1_lower = args.color1_hsv[0:3]
        color1_upper = args.color1_hsv[3:6]
    else:
        if args.color1_lower:
            color1_lower = args.color1_lower
        if args.color1_upper:
            color1_upper = args.color1_upper
    
    if args.color2_hsv:
        color2_lower = args.color2_hsv[0:3]
        color2_upper = args.color2_hsv[3:6]
    else:
        if args.color2_lower:
            color2_lower = args.color2_lower
        if args.color2_upper:
            color2_upper = args.color2_upper
    
    color1 = ColorDescription(color1_name,color1_lower,color1_upper)
    color2 = ColorDescription(color2_name,color2_lower,color2_upper)

    red_lower = RED_LOWER
    red_upper = RED_UPPER
    if args.red_hsv:
        red_lower = args.red_hsv[0:3]
        red_upper = args.red_hsv[3:6]
    else:
        if args.red_lower:
            red_lower = args.red_lower
        if args.red_upper:
            red_upper = args.red_upper

    if args.ROI_size:
        ROI = ROI_size(*args.ROI_size)
    else:
        ROI = DEFAULT_ROI_SIZE
    del args.ROI_size
    
    red_ROI_size = ROI_size(*args.red_ROI_size) if args.red_ROI_size else DEFAULT_RED_ROI_SIZE
    
    for name in dir(args):
        if name.startswith('color') or name.startswith('red'):
            delattr(args,name)
    
    kwargs = vars(args).copy()
    kwargs['red'] = ColorDescription('red',red_lower,red_upper)
    kwargs['red_ROI_size'] = red_ROI_size
    
    gs = ColorSegmenter(arm, camera_name, ROI,
                        color1=color1, color2=color2, **kwargs)

    """
    gs = ColorSegmenter('R', left_camera, right_camera,
                        color1=('orange',ORANGE_LOWER,ORANGE_UPPER),
                        color2=('green',GREEN_LOWER,GREEN_UPPER),
                        **vars(args))
    gs = ColorSegmenter('L', left_camera, right_camera,
                        color2=('purple',PURPLE_LOWER,PURPLE_UPPER),
                        color1=('blue',BLUE_LOWER,BLUE_UPPER),
                        **vars(args))
    """
    rospy.spin()


def test_cache():
    rospy.init_node('test_cache',anonymous=True)
    rospy.sleep(1)
    listener = tf.TransformListener()
    f0 = '/left_optical_frame'
    f1 = '/0_link'
    while not rospy.is_shutdown():
        try:
            common = listener.getLatestCommonTime(f0, f1)
            listener.waitForTransform(f0,f1,common,rospy.Duration(4.0))
            break
        except tf.Exception:
            rospy.loginfo('tf exception')

        rospy.sleep(.1)
        
    rospy.loginfo('found it!')

if __name__ == '__main__':
    main()
    

