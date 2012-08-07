import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("raven_2")
from raven_2.msg import raven_state, joint_command
import rospy
import sensor_msgs.msg as sm

rospy.init_node("test_joint_publishing")
print "getting joint state"
joint_state_msg = rospy.wait_for_message("joint_states", sm.JointState, timeout = 4)

assert isinstance(joint_state_msg, sm.JointState)

torque = [0 for _ in xrange(8)]
torque[4] +=.03

pub = rospy.Publisher("joint_cmd1", joint_command)
rospy.sleep(.5)

msg = joint_command(torque = torque)
pub.publish(msg)

print "done!"
