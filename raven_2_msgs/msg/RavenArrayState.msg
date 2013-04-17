Header header
uint8 runlevel
uint8 sublevel
bool pedal_down

string master
uint8 controller #see Constants.msg for values

# ****** ARMS ******

string[] arm_names
uint8[] arm_types #see Constants.msg for values

geometry_msgs/Pose[] base_poses

# ****** TOOLS ******

uint8[] tool_types #see Constants.msg for values

ToolArrayState tool

ToolSetPointArray tool_set_points

# ****** JOINTS ******

uint8[] joint_arm_inds
uint16[] joint_types #see Constants.msg for values
int16[] joint_states # from JointState.msg

int32[] motor_encoder_values  # encoder value
int32[] motor_encoder_offsets # Encoder offset to "zero"

float32[] motor_positions
float32[] motor_velocities

float32[] joint_positions  # actual DOF coordinate (rad)
float32[] joint_velocities # actual DOF velocity(q-dot)

float32[] torques       # commanded DOF force/torque
int16[] dac_commands    # DAC command to achieve tau at actuator

float32[] gravity_estimates # Estimated gravity force/torque on joint.

uint8[] joint_command_types # see JointCommands.msg for values
float32[] joint_commands

float32[] integrated_position_errors     # integrated position error for joint space position control

uint8[] input_pins
uint8[] output_pins

DOFSetPointArray set_points
