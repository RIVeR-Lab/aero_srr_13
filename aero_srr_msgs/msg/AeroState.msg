#Head info
Header header

#The current robot mission satate
uint8 state

#Constants defining the valid values for state
##############################################
#The Robot is currently in a fatal error state
uint8 ERROR=0

#The Robot is currently searching the field
uint8 SEARCH=1

#The robot is currently navigating to an object of interest
uint8 NAVOBJ=2

#The robot is currently collecting an object of interest
uint8 COLLECT=3

#The robot is currently homeing to the start platform
uint8 HOME=4

#The robot is currently paused
uint8 PAUSE=5

#The robot is currently safe-estoping
uint8 SAFESTOP=6

#The robot is currently shutting down
uint8 SHUTDOWN=7

#The robot is currenlty in manual mode
uint8 MANUAL=8

#The robot is currently in startup mode
uint8 STARTUP=9

#The robot is currently picking up an object of interest
uint8 PICKUP=10
##############################################
