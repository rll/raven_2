import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("raven_2")
from raven_2.msg import raven_state, joint_command
import rospy
import sensor_msgs.msg as sm

from enthought.traits.api import *
from enthought.traits.ui.api import *

class JointTorques(HasTraits):
    j0 = Range(-.1,.1,0)
    j1 = Range(-.1,.1,0)
    j2 = Range(-.1,.1,0)
    j3 = Range(-.1,.1,0)
    j4 = Range(-.1,.1,0)
    j5 = Range(-.1,.1,0)
    j6 = Range(-.1,.1,0)
    j7 = Range(-.1,.1,0)
    j8 = Range(-.1,.1,0)
    j9 = Range(-.1,.1,0)
    j10 = Range(-.1,.1,0)
    j11 = Range(-.1,.1,0)
    j12 = Range(-.1,.1,0)
    j13 = Range(-.1,.1,0)
    j14 = Range(-.1,.1,0)
    j15 = Range(-.1,.1,0)

    traits_view = View(VGroup(
            Item("j0"),
            Item("j1"),
            Item("j2"),
            Item("j3"),
            Item("j4"),
            Item("j5"),
            Item("j6"),
            Item("j7"),
            Item("j8"),
            Item("j9"),
            Item("j10"),
            Item("j11"),
            Item("j12"),
            Item("j13"),
            Item("j14"),
            Item("j15")))

    def __init__(self):
        HasTraits.__init__(self)
        self.pub = rospy.Publisher("joint_cmd1", joint_command)
        rospy.sleep(.5)
        self.on_trait_change(self.publish_torques)

    def publish_torques(self):
        msg = joint_command(
            torque = [self.j0, self.j1, self.j2, self.j3, self.j4, self.j5, self.j6, self.j7, self.j8, self.j9, self.j10, self.j11, self.j12, self.j13, self.j14, self.j15])
        self.pub.publish(msg)
        print "published!"
        
        
rospy.init_node("joint_torque_gui")
JointTorques().configure_traits()
