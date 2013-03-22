#Head info
Header header

#object type
uint8 object_type

#Constants defining the valid values for state
##############################################
uint8 PINK_BALL=0

uint8 WHITE_HOOK=1

uint8 ORANGE_PIPE=2

uint8 WOODEN_BLOCK=3
##############################################

#Location of the object
geometry_msgs/Point point

#Confidence within location
float64 confidence 

