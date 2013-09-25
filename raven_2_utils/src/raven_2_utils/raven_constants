#!/usr/bin/env python

"""
Contains general constants that
are frequently used
"""

# Import required Python code.
import roslib
roslib.load_manifest('raven_2_utils')
import rospy

class StereoClick:
    StereoName = 'stereo_points_3d'

class AR:
    Stereo = 'stereo_pose'
    class Frames:
        Grasper1 = '/grasper1_tip'
        Grasper2 = '/grasper2_tip'
        Right = '/grasper_r'
        Cube1 = '/cube_1'
        Cube2 = '/cube_2'
        Cube3 = '/cube_3'
        Cube4 = '/cube_4'
        Object = '/object'
        Base = '/0_link'

class Arm:
    Left = 'L'
    Right = 'R'
    Both = [Left,Right]

class Frames:
    Camera = 'left_optical_frame'
    LeftTool = 'tool_L'
    RightTool = 'tool_R'
    World = 'world'
    LeftBase = 'base_link_L'
    RightBase = 'base_link_R'
    Link0 = '0_link'
    
class OpenraveLinks:
    LeftWrist = 'wrist_L'
    RightWrist = 'wrist_R'
    LeftTool = 'tool_L'
    RightTool = 'tool_R'
    LeftToolBase = 'tool_base_L'
    RightToolBase = 'tool_base_R'
    LeftToolTip = 'tool_tip_L'
    RightToolTip = 'tool_tip_R'

class RavenTopics:
    #publish ToolCommandStamped directly
    LeftTool = '/raven_command/tool/L'
    RightTool = '/raven_command/tool/R'

    RavenState = '/raven_state'
    RavenCommand = '/raven_command'

class Foam:
    Topic = '/foam_points'

class GripperTape:
    Topic = '/tape_pose'

class Services:
    isFoamGrasped = 'thresh_red'
