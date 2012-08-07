#!/usr/bin/env python
import roslib; roslib.load_manifest("raven_2_trajectory")
import rospy
import rosbag
import math, time
import tf
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from tf.msg import tfMessage
import sys

from raven_2_trajectory.srv import *

BASE_FRAME = '/0_link'
END_EFFECTOR_FRAME = '/tool_L'

tools = ['L'] #lr = ['L','R']

class TrajectoryRecorder:
    
    def __init__(self,listener):
        self.listener = listener
        print "waiting for transform from {0} to {1}".format(BASE_FRAME, END_EFFECTOR_FRAME)
        tries = 0
        while not rospy.is_shutdown():
            tries += 1
            #print "Try #%d" % tries
            try:
                self.listener.waitForTransform(BASE_FRAME, END_EFFECTOR_FRAME, rospy.Time(0), rospy.Duration(5.0))
                break
            except tf.Exception, e:
                continue
        print "got it!"

        self.bag = None
        self.active = False
        self.default_fileprefix = "traj"
        self.current_fileprefix = None
        
        self.service = rospy.Service("record_trajectory",RecordTrajectory,self.service_callback)
        
        self.joint_state = None
        self.joint_state_sub = rospy.Subscriber("/joint_states",JointState,self.joint_callback)

        self.tf = None
        self.tf_sub = rospy.Subscriber("/tf",tfMessage,self.tf_callback)

        self.tool_pose = {}
        self.tool_pose_sub = {}
        for tool in tools:
            self.tool_pose_sub[tool] = rospy.Subscriber("tool_pose_%s" % tool,PoseStamped,lambda msg: self.tool_pose_callback(msg,tool))

        print "let's do this!"
    
    def getFilename(self,prefix):
        if not prefix:
            prefix = self.default_fileprefix
        self.current_fileprefix = prefix
        return prefix + "_" + time.strftime("%Y_%m_%d_T%H_%M_%S") + ".bag"

    def service_callback(self,req):
        try:
            if req.activate:
                already_active = self.active
                if self.active and req.new_bag and self.bag:
                    print "Closing current bag %s" % self.bag.filename
                    self.close()
                if not self.bag or (req.filename and req.filename != self.current_fileprefix):
                    print "Activating..."
                    fname = self.getFilename(req.filename)
                    self.bag = rosbag.Bag(fname,'w')
                    print "Writing to %s" % self.bag.filename
                self.active = True
                if not already_active:
                    return RecordTrajectoryResponse(RecordTrajectoryResponse.SUCCESS)
                else:
                    return RecordTrajectoryResponse(RecordTrajectoryResponse.NO_CHANGE)
            elif self.active:
                print "Deactivating..."
                self.close()
                self.active = False
                self.current_fileprefix = None
                return RecordTrajectoryResponse(RecordTrajectoryResponse.SUCCESS)
            return RecordTrajectoryResponse(RecordTrajectoryResponse.NO_CHANGE)
        except Exception, e:
            print "Exception: %s" % e
            return RecordTrajectoryResponse(RecordTrajectoryResponse.ERROR)

    def joint_callback(self,msg):
        self.joint_state = msg

    def tf_callback(self,msg):
        self.tf = msg

    def tool_pose_callback(self,msg,tool):
        self.tool_pose[tool] = msg

    def close(self):
        if self.bag is not None:
            self.bag.close()
            print "Closed bag %s" % self.bag.filename
            self.bag = None
    
    def write(self):
        if not self.active: return
        try:
            #print "writing..."
            self.bag.write('/joint_states',self.joint_state)
            self.bag.write('/tf',self.tf)
            for tool in tools:
                self.bag.write("tool_pose_%s" % tool,self.tool_pose[tool])
            now = self.joint_state.header.stamp
            for side in lr:
                #(trans,rot) = self.listener.lookupTransform('/0_link', '/tool_'+side, rospy.Time(0))
                (trans,rot) = self.listener.lookupTransform('/0_link', '/tool_'+side, now)
                tool_pose = PoseStamped()
                tool_pose.header.stamp = now
                tool_pose.header.frame_id = "/0_link"
                tool_pose.pose.position = Point(*trans)
                tool_pose.pose.orientation = Quaternion(*rot)
                self.bag.write('tool_pose_'+side,tool_pose)
                self.pub.publish(tool_pose)
        except (tf.LookupException, tf.ConnectivityException), e:
            print "exception!",e

if __name__ == "__main__":
    rospy.init_node("raven_2_trajectory_recorder")

    listener = tf.TransformListener()

    tr = TrajectoryRecorder(listener)

    if len(sys.argv) > 1:
        tr.default_fileprefix = sys.argv[1]

    try:
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                tr.write()
            except Exception, e:
                print "Exception while writing: ",e
            rate.sleep()
    finally:
        tr.close()
    
    
