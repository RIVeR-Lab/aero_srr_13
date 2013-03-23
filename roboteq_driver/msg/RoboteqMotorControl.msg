# control a single channel of the controller

# Control Mode Options
uint8 POWER =1
uint8 RPM = 2
uint8 POSITION = 3

# the channel on the controller to control (1 or 2)
uint16 channel
# how to interpert the setpoint
uint8 control_mode
int32 setpoint
