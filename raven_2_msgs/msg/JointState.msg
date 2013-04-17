uint16 type #see Constants.msg for values

int16 state # is this DoF enabled?
int16 STATE_NOT_READY   = 0
int16 STATE_POS_UNKNOWN = 1
int16 STATE_HOMING1     = 2
int16 STATE_HOMING2     = 3
int16 STATE_READY       = 4
int16 STATE_WAIT        = 5
int16 STATE_HARD_STOP   = 6
int16 STATE_LAST_TYPE   = 7

string STATE_STRINGS = NOT_READY,POS_UNKNOWN,HOMING1,HOMING2,READY,WAIT,HARD_STOP,LAST_TYPE

int32 encoder_value	  # encoder value
int32 encoder_offset  # Encoder offset to "zero"
int16 dac_command	  # DAC command to achieve tau at actuator

float32 position		# actual DOF coordinate (rad)
float32 velocity 		# actual DOF velocity(q-dot)

float32 motor_position
float32 motor_velocity

float32 torque           # commanded DOF force/torque
float32 gravity_estimate # Estimated gravity force/torque on joint.

JointCommand command

float32 integrated_position_error     # integrated position error for joint space position control

DOFSetPoint set_point
