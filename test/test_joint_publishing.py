import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("raven_2")
from raven_2.msg import raven_state, joint_command
import rospy
import sensor_msgs.msg as sm

rospy.init_node("test_joint_publishing")
print "getting raven state"
raven_state_msg = rospy.wait_for_message("raven_state", raven_state, timeout = rospy.Duration(10))

assert isinstance(raven_state_msg, raven_state)
arm1_joints = raven_state_msg.jpos[0:8]

print "arm1 joints", arm1_joints

arm1_joints[0] += .1

pub = rospy.Publisher("joint_cmd1", joint_command)
msg = joint_command(jpos = arm1_joints)
pub.publish(msg)

print "done!"
