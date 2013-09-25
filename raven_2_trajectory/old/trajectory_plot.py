import roslib; roslib.load_manifest('raven_2_trajectory')
import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from raven_2_msgs.msg import *

def plot_joint_field(j,field,prefix=None):
	name = field
	if prefix:
		name = prefix + ': ' + name
	#plt.figure(name.__hash__() % 1000)
	plt.figure(name)
	plt.clf()
	
	
	if field == 'position_diff':
		data = j.xs('position_set_point',axis=1,level='field') - j.xs('position',axis=1,level='field')
	else:
		data = j.xs(field,axis=1,level='field')
	
	index = data.index
	index = [(i - index[0]).total_seconds() for i in index]
	
	plt.plot(index,data)
	types = j.xs('type',axis=1,level='field').ix[0,:]
	plt.legend([types.index[i][0] +' '+ Constants.JOINT_TYPE_STRINGS.split(',')[int(types[i])] for i in xrange(len(types))],loc=2)
	plt.title(name)
	plt.xlabel('Time (s)')

def plot_all_joint_fields(j,prefix=None):
	fields = {'position':'rad','velocity':'rad/s','motor_position':'rad/s','motor_velocity':'rad/s','torque':'','position_set_point':'rad'}
	
	for field,units in fields.iteritems():
		plot_joint_field(j,field,prefix)
		plt.ylabel(field + ' ' + units)
	
	plot_joint_field(j,'state',prefix)
	plt.gca().set_yticklabels(JointState.STATE_STRINGS.split(','))
	plt.draw()