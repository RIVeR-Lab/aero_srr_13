//============================================================================
// Name        : OryxManager.h
// Author      : Jon Anderson jonjake
// Version     : 1.0
// Date		   : Nov 17, 2011
// Copyright   : Worcester Polytechnic Institute
// Project	   : Oryx 2.0 (www.wpirover.com)
// Description :
//============================================================================

#ifndef ORYXMANAGER_H_
#define ORYXMANAGER_H_
using namespace std;
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/TwistStamped.h"
#include <OryxMessages/Battery.h>
#include <OryxMessages/Temperature.h>
#include <EposManager/GroupEPOSControl.h>
#include <EposManager/EPOSControl.h>
#include <EposManager/GroupMotorInfo.h>
#include <EposManager/MotorInfo.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>


//Define EPOS Nodes
#define FRONT_LEFT_WHEEL 1
#define BACK_LEFT_WHEEL 2
#define FRONT_RIGHT_WHEEL 3
#define BACK_RIGHT_WHEEL 4

#define PTZ_PAN 3
#define PTZ_TILT 2
#define PTZ_BOOM 1

#define ARM_SHOULDER 1
#define ARM_SCOOP 2
#define ARM_PAN 3

//Define Kinematics
#define WHEEL_DIAMETER (12.5*2.54/100)
#define TICS_PER_MOTOR_REV 1024
#define GEAR_RATIO 156
#define WHEEL_BASE 0.86147
#define ROCKER_LENGTH .33020
#define ROCKER_OFFSET 0.148352986 //radians below horizontal
#define WHEEL_TRACK 100

//Define Temperature Nodes
#define COMPUTER_BOX_TEMP_NODE 0
#define CPU_CORE_1_TEMP_NODE 1
#define CPU_CORE_2_TEMP_NODE 2
#define CPU_CORE_3_TEMP_NODE 3
#define CPU_CORE_4_TEMP_NODE 4
#define BMS_TEMP_NODE 5
#define BATTERY_CELL_1_NODE 6
#define BATTERY_CELL_2_NODE 7
#define BATTERY_CELL_3_NODE 8
#define BATTERY_CELL_4_NODE 9
#define BATTERY_CELL_5_NODE 10
#define BATTERY_CELL_6_NODE 11
#define FRONT_LEFT_WHEEL_TEMP_NODE 12
#define BACK_LEFT_WHEEL_TEMP_NODE 13
#define FRONT_RIGHT_WHEEL_TEMP_NODE 14
#define BACK_RIGHT_WHEEL_TEMP_NODE 15



//Define joystick axes/buttons
#define LEFT_HORIZONTAL_AXIS 0
#define LEFT_VERTICAL_AXIS 1
#define LEFT_TRIGGER 2
#define RIGHT_HORIZONTAL_AXIS 3
#define RIGHT_VERTICAL_AXIS 4
#define RIGHT_TRIGGER 5
#define D_PAD_LEFT_RIGHT 6
#define D_PAD_UP_DOWN 7


#define A_BUTTON 0
#define B_BUTTON 1
#define X_BUTTON 2
#define Y_BUTTON 3
#define LEFT_BUTTON 4
#define RIGHT_BUTTON 5
#define SELECT_BUTTON 6
#define START_BUTTON 7
#define CENTER_BUTTON 8
#define LEFT_STICK 9
#define RIGHT_STICK 10


struct homingData{
	int nodeId;
	long int minValue;
	long int maxValue;
	long int homePos;
	int homeVelocity;
	int homeCurrent;
	bool isHoming;
	bool isReadyForControl;
	int ignoreCounter;
	long int currentPosition;
	long int timesToIgnore;
	double angle;
};


#endif /* ORYXMANAGER_H_ */
