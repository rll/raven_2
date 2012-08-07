import roslib
roslib.load_manifest("sensor_msgs")
roslib.load_manifest("rospy")
import numpy as np
import openravepy
import sensor_msgs.msg as sm

env = rave.Environment()
env.SetViewer('qtcoin')
env.Load("/home/biorobotics/Data/raven.dae")
raven = env.GetRobots()[0]


def update(jointstate):
  rosvalues = jointstate.position
  rosnames = jointstate.name 

  ros2rave = np.array([raven.GetJointIndex(name) for name in rosnames])
  goodrosinds = np.flatnonzero(ros2rave != -1)
  raveinds = ros2rave[goodrosinds]
  ravevalues = rosvalues[goodrosinds]
  raven.SetJointValues(ravevalues, raveinds)

  
rospy.init_node("openrave_viz")
sub = rospy.Subscriber("/jointstates", sm.JointState, update)