uint8 command_type
float32 value

uint8 COMMAND_TYPE_POSITION=0 # desired DOF coordinate (rad)
uint8 COMMAND_TYPE_VELOCITY=1 # desired DOF velocity (q-dot-desired)

uint8 COMMAND_TYPE_MOTOR_POSITION=2
uint8 COMMAND_TYPE_MOTOR_VELOCITY=3

uint8 COMMAND_TYPE_TORQUE=4          # desired DOF force/torque
