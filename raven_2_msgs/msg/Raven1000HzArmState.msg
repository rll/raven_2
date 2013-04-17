int16[] joint_states

int32[] motor_encoder_values  # encoder value
int32[] motor_encoder_offsets # Encoder offset to "zero"

float32[] motor_positions
float32[] motor_velocities

float32[] joint_positions  # actual DOF coordinate (rad)
float32[] joint_velocities # actual DOF velocity(q-dot)

float32[] torques # commanded DOF force/torque
int16[] dac_commands

float32[] gravity_estimates # Estimated gravity force/torque on joint.
float32[] integrated_position_errors

DOFSetPointArray set_points
