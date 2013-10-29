#!/usr/bin/env python
import roslib
roslib.load_manifest('raven_2_vision')
import rospy
import pickle
import tf
import tf.transformations as tft
import tfx
import os.path 

from raven_2_msgs.msg import *
from geometry_msgs.msg import PoseStamped, TransformStamped

from raven_2_utils import raven_util
from raven_2_utils import raven_constants

from raven_2_control import kinematics

import IPython

from collections import defaultdict
from functools import partial

class GripperPoseEstimator():
    """
    Used to estimate gripper pose by image processing
    """

    def __init__(self, arms = ['L','R'], calcPosePostAdjustment=None, adjustmentInterpolation=True,systematicError = True):
        self.arms = arms
        self.useSystematicError = systematicError 
        self.truthPose = {}
        self.calcPose = {}
        self.calcPoseAtTruth = {}
        self.sys_error = {}
        self.sys_error= {}
        self.est_pose_pub = {}
        self.pose_error_pub = {}
        self.estimateFromImage = dict()
        self.estimatedPose = defaultdict(lambda: (None,None))
        if systematicError:
            trained_data_R = pickle.load(open(os.path.join(roslib.packages.get_pkg_subdir('raven_2_params','data/right_arm'),'right_train_data.pkl')))
            trained_data_L =  pickle.load(open(os.path.join(roslib.packages.get_pkg_subdir('raven_2_params','data/left_arm'),'left_train_data.pkl')))
            self.sys_error['R'] = tfx.transform(trained_data_R["sys_robot_tf"]['R'])
            self.sys_error['L'] = tfx.transform(trained_data_L["sys_robot_tf"]['L'])
            print self.sys_error['R']
            print self.sys_error['L']
        else:    
            self.pre_adjustment = dict((arm,tfx.identity_tf()) for arm in self.arms)
            self.post_adjustment = dict((arm,tfx.identity_tf()) for arm in self.arms)
            self.adjustment_side = dict((arm,'post') for arm in self.arms)
            self.switch_adjustment_update = True
            self.adjustmentInterpolation = {'pre': 1, 'post': 1}
            if adjustmentInterpolation is True:
                self.adjustmentInterpolation['pre'] = self.adjustmentInterpolation['post'] = 0.5
            elif isinstance(adjustmentInterpolation,dict):
                self.adjustmentInterpolation['pre'] = adjustmentInterpolation.get('pre',0.5)
                self.adjustmentInterpolation['post'] = adjustmentInterpolation.get('post',0.5)
            elif isinstance(adjustmentInterpolation,(list,tuple)):
                self.adjustmentInterpolation['pre'] = adjustmentInterpolation[0]
                self.adjustmentInterpolation['post'] = adjustmentInterpolation[1]
            else:
                self.adjustmentInterpolation['pre'] = self.adjustmentInterpolation['post'] = adjustmentInterpolation
        
            self.calcPosePostAdjustment = dict((arm,tfx.identity_tf()) for arm in self.arms)
            if calcPosePostAdjustment:
                for k,v in calcPosePostAdjustment:
                    self.calcPosePostAdjustment[k] = tfx.transform(v,copy=True)
        
            self.pre_adj_pub = {}
            self.post_adj_pub = {}
            for arm in self.arms:
                rospy.Subscriber(raven_constants.GripperTape.Topic+'_'+arm, PoseStamped, partial(self._truthCallback,arm))
                self.pre_adj_pub[arm] = rospy.Publisher('estimated_gripper_pose_pre_adjument_%s'%arm, TransformStamped)
                self.post_adj_pub[arm] = rospy.Publisher('estimated_gripper_pose_post_adjument_%s'%arm, TransformStamped)
                
        for arm in self.arms:
            self.est_pose_pub[arm] = rospy.Publisher('estimated_gripper_pose_%s'%arm, PoseStamped)
            self.pose_error_pub[arm] = rospy.Publisher('estimated_gripper_pose_error_%s'%arm, PoseStamped)    
        rospy.Subscriber(raven_constants.RavenTopics.RavenState, RavenState, self._ravenStateCallback)
          
    
    def _truthCallback(self,arm,msg):
        if not self.estimateFromImage.get(arm, True):
            return
        
        #rospy.loginfo("%f",msg.header.stamp.to_sec())
        try:
            #rospy.loginfo('looking up transform')
            tf_msg_to_link0 = tfx.lookupTransform(raven_constants.Frames.Link0, msg.header.frame_id, wait=5)
            truthPose = tf_msg_to_link0 * tfx.pose(msg)
            
            #rospy.loginfo('found transform')
            #truthPose = tfx.convertToFrame(msg, raven_constants.Frames.Link0, ignore_stamp=True)
        except Exception, e:
            print e
            raise e
        #truthPose = raven_util.convertToFrame(msg, raven_constants.Frames.Link0)
        calcPose = self.calcPose.get(arm)
        
        prevTruthPose = self.truthPose.get(arm)
        prevCalcPose = self.calcPoseAtTruth.get(arm)
        
        if prevTruthPose is not None and prevCalcPose is not None:
            currEstPose = self.estimatedPose[arm][0]
            poseError = (truthPose.as_tf().inverse() * currEstPose)
            self.pose_error_pub[arm].publish(poseError.msg.PoseStamped(stamp=truthPose.stamp))
        
            deltaTruthPose = prevTruthPose.as_tf().inverse() * truthPose.as_tf()
            deltaCalcPose = prevCalcPose.as_tf().inverse() * calcPose.as_tf()
            
            prev_pre_adjustment = self.pre_adjustment[arm]
            prev_post_adjustment = self.post_adjustment[arm]
            
            if self.adjustment_side[arm] == 'pre':
                adjustment = deltaTruthPose * (deltaCalcPose * prev_post_adjustment).inverse()
                adjustment.orientation = tfx.tb_angles(0,0,0)
                adjustment.translation = [0,0,0]
                self.pre_adjustment[arm] = self.pre_adjustment[arm].interpolate(adjustment,self.adjustmentInterpolation['pre'])
                if self.switch_adjustment_update:
                    self.adjustment_side[arm] = 'post'
            elif self.adjustment_side[arm] == 'post':
                adjustment = (prev_pre_adjustment * deltaCalcPose).inverse() * deltaTruthPose
                #adjustment.translation = [0,0,0]
                self.post_adjustment[arm] = self.post_adjustment[arm].interpolate(adjustment,self.adjustmentInterpolation['post'])
                if self.switch_adjustment_update:
                    self.adjustment_side[arm] = 'pre'
            
            self.pre_adj_pub[arm].publish(self.pre_adjustment[arm].msg.TransformStamped(stamp=truthPose.stamp, check_frame=False))
            self.post_adj_pub[arm].publish(self.post_adjustment[arm].msg.TransformStamped(stamp=truthPose.stamp, check_frame=False))
    
        self.truthPose[arm] = truthPose
        self.calcPoseAtTruth[arm] = calcPose
        self.estimatedPose[arm] = (truthPose,False)
    i =1    
    def _ravenStateCallback(self,msg):
        
        if self.useSystematicError:
            
            for arm in self.arms:
                if self.estimatedPose.has_key(arm):
                    if tfx.stamp(msg.header.stamp) - self.estimatedPose[arm][0].stamp < 0.2:
                        return
                
                arm_msg = [msg_arm for msg_arm in msg.arms if msg_arm.name == arm][0]           
                joints = dict((j.type,j.position) for j in arm_msg.joints)
                fwdArmKinPose, grasp = kinematics.fwdArmKin(arm,joints,stamp=msg.header.stamp)
                estPose = tfx.pose(self.sys_error[arm] * fwdArmKinPose,frame='0_link',stamp = msg.header.stamp)
                estPose.frame = '0_link' # TEMP
                self.estimatedPose[arm] = (estPose,False)
             
                self.est_pose_pub[arm].publish(estPose.msg.PoseStamped())

        else:
            if self.calcPose:
                prevTime = self.calcPose.values()[0].stamp
                if tfx.stamp(msg.header.stamp).seconds - prevTime.seconds < 0.2:
                    return
            for arm in self.arms:
                arm_msg = [msg_arm for msg_arm in msg.arms if msg_arm.name == arm][0]
                #self.calcPose[arm] = tfx.pose(arm_msg.tool.pose,header=msg.header)
            
                joints = dict((j.type,j.position) for j in arm_msg.joints)
                fwdArmKinPose, grasp = kinematics.fwdArmKin(arm,joints)
                self.calcPose[arm] = (fwdArmKinPose.as_tf() * self.calcPosePostAdjustment[arm]).as_pose(stamp=msg.header.stamp)
            
                self._updateEstimatedPose(arm)
    
    def _updateEstimatedPose(self,arm):
        calcPose = self.calcPose[arm]
        prevTruthPose = self.truthPose.get(arm)
        if prevTruthPose is None:
            return
        prevCalcPose = self.calcPoseAtTruth[arm]
        if prevCalcPose is None:
            return
        pre_adjustment = self.pre_adjustment[arm]
        post_adjustment = self.post_adjustment[arm]
        
        deltaCalcPose = prevCalcPose.inverse().as_tf() * calcPose.as_tf()
        deltaPose = pre_adjustment * deltaCalcPose * post_adjustment
        estPose = tfx.pose(prevTruthPose.as_tf() * deltaPose,frame=calcPose.frame,stamp=calcPose.stamp)
        self.est_pose_pub[arm].publish(estPose.msg.PoseStamped())
        self.estimatedPose[arm] = (estPose,True)
        
       
    
    def getGripperPose(self, armName, full=False):
        """
        armName must be from raven_constants.Arm
        """
        pose, isEst = self.estimatedPose[armName]
        if full:
            return pose, isEst
        else:
            return pose
        
    def suppressImageEstimation(self, armName):
        self.estimateFromImage[armName] = False
        
    def enableImageEstimation(self, armName):
        self.estimateFromImage[armName] = True
        
def test():
    rospy.init_node('testGripperPoseEstimator',anonymous=True)
    rospy.sleep(2)
    arms = raven_constants.Arm.Both
    gpe = GripperPoseEstimator(arms)
    rospy.sleep(2)
    
    estimatePub = dict()
    truthPub = dict()
    prevPoseAndIsEstimate = dict()
    
    for arm in arms:
        truthPub[arm] = rospy.Publisher('truth_gripper_pose_%s'%arm,PoseStamped)
        
        
    printEvery = raven_util.Timeout(1)
    printEvery.start()
    
    while not rospy.is_shutdown():
        for arm in arms:
            truthPose = gpe.truthPose.get(arm)
            estimatedPoseisEstimate = gpe.estimatedPose.get(arm)
            
            if truthPose is not None:
                #print 'Publishing truthPose %s' % arm
                truthPub[arm].publish(truthPose.msg.PoseStamped())
            if estimatedPoseisEstimate is not None:
                estimatedPose, isEstimate = estimatedPoseisEstimate
                #print 'Publish estimatedPose %s' % arm
                
            if estimatedPoseisEstimate is not None and prevPoseAndIsEstimate.has_key(arm):
                prevPose, prevIsEstimate = prevPoseAndIsEstimate[arm]
                if prevIsEstimate is True and isEstimate is False:
                    deltaPose = raven_util.deltaPose(prevPose, estimatedPose)
                    print 'deltaPose between last estimate and current truth'
                    print deltaPose
                #print 'truthPose_{0}: {1}'.format(arm,truthPose)
                #print 'isEstimate: {0}'.format(isEstimate)
                #print 'estimatedPose_{0}: {1}'.format(arm,estimatedPose)
            
            if estimatedPoseisEstimate is not None:
                prevPoseAndIsEstimate[arm] = (estimatedPose, isEstimate)
        
        rospy.sleep(.02)
        
def testTimeStamps():
    rospy.init_node('testTimeStampes',anonymous=True)
    rospy.sleep(1)
    arms = raven_constants.Arm.Both
    gpe = GripperPoseEstimator(arms)
    rospy.sleep(2)
    
    def _estimatePoseCallback(msg):
        rospy.loginfo('Received estimated gripper pose in MoveTowardsReceptacle')
        print 'pose'
        print tfx.pose(msg)
        print 'time'
        print tfx.pose(msg).stamp.seconds
        print 'now - msg time'
        print rospy.Time.now().to_sec() - tfx.pose(msg).stamp.seconds
    
    rospy.Subscriber('estimated_gripper_pose_R',PoseStamped,_estimatePoseCallback)
    rospy.spin()

def standalone_main():
    rospy.init_node('standaloneGripperPoseEstimator',anonymous=True)
    gpe = GripperPoseEstimator()
    rospy.spin()

if __name__ == '__main__':
    standalone_main()
    #testTimeStamps()
      
