# This represents the motor information of a specific motor
# node_id is the node_id set on the EPOS Controller itself
# motor_name is the name of the motor, as assigned by the user
# enabled indicated whether or not the motor is enabled or disabled
# motor_mode is the mode the motor is in (velocity, position, current)
# motor_velocity is the velocity of the motor in rpm
# motor_position is the position of the motor in encoder counts
# motor_current is the current on the motor in milliamps

uint16 node_id
string motor_name
uint16 state
string faults
int32 motor_velocity
int32 motor_position
int32 motor_current
time stamp
