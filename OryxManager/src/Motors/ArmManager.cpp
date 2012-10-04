/*
 * ArmManager.cpp
 *
 *  Created on: Jan 26, 2012
 *      Author: oryx
 */

#include "ArmManager.h"
#define topSpeed 2000

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
	if (msg->buttons[RIGHT_BUTTON] >.1) {
		//Control velocities with joysticks
		float scale = 1;
		if ((msg->axes[LEFT_TRIGGER] >= -.5) || (msg->axes[RIGHT_TRIGGER]>= -.5))scale = .5;
		if ((msg->axes[LEFT_TRIGGER] >= -.5) && (msg->axes[RIGHT_TRIGGER]	>= -.5))scale = .125;
		if (msg->axes[LEFT_VERTICAL_AXIS] < 0){
			sendVelocityMessage(shoulderMotor,(int)(msg->axes[LEFT_VERTICAL_AXIS]*-shoulderMotor.homeVelocity*scale));
		}
		else if (msg->axes[LEFT_VERTICAL_AXIS] >= 0){
			 sendVelocityMessage(shoulderMotor,(int)(msg->axes[LEFT_VERTICAL_AXIS]*-shoulderMotor.homeVelocity*scale));
		}
		if (msg->axes[LEFT_HORIZONTAL_AXIS] < 0){
			sendVelocityMessage(panMotor,(int)(msg->axes[LEFT_HORIZONTAL_AXIS]*panMotor.homeVelocity*scale));
		}
		else if (msg->axes[LEFT_HORIZONTAL_AXIS] >= 0){
			sendVelocityMessage(panMotor,(int)(msg->axes[LEFT_HORIZONTAL_AXIS]*panMotor.homeVelocity*scale));
		}
		if(msg->buttons[SELECT_BUTTON] > .1 || msg->buttons[START_BUTTON] ){
			if(msg->buttons[SELECT_BUTTON] > .1){
				sendVelocityMessage(scoopMotor,(int)(-scoopMotor.homeVelocity*scale));
			}
			else sendVelocityMessage(scoopMotor,(int)(scoopMotor.homeVelocity*scale));
		}
		else sendVelocityMessage(scoopMotor,(int)(0));
		if(msg->buttons[CENTER_BUTTON] > .1){
			if(!shoulderMotor.isHoming)dropOffRock();
		}
	}
	else {
		sendVelocityMessage(panMotor,0);
		sendVelocityMessage(shoulderMotor,0);
		sendVelocityMessage(scoopMotor,0);
	}
}

void driverJoyCallback(const sensor_msgs::Joy::ConstPtr& msg){
	//If the Left or Right Button is Pressed
	if (msg->buttons[RIGHT_BUTTON] >.1 || msg->buttons[LEFT_BUTTON] >.1) {
			//Control velocities with joysticks
		float scale = 1;
		if ((msg->axes[LEFT_TRIGGER] >= -.5) || (msg->axes[RIGHT_TRIGGER]>= -.5))scale = .5;
		if ((msg->axes[LEFT_TRIGGER] >= -.5) && (msg->axes[RIGHT_TRIGGER]	>= -.5))scale = .125;
				if (msg->axes[LEFT_VERTICAL_AXIS] < 0){
					sendVelocityMessage(shoulderMotor,(int)(msg->axes[LEFT_VERTICAL_AXIS]*scale*-shoulderMotor.homeVelocity));
				}
				else if (msg->axes[LEFT_VERTICAL_AXIS] >= 0){
					 sendVelocityMessage(shoulderMotor,(int)(msg->axes[LEFT_VERTICAL_AXIS]*scale*-shoulderMotor.homeVelocity));
				}
				if (msg->axes[LEFT_HORIZONTAL_AXIS] < 0){
					sendVelocityMessage(panMotor,(int)(msg->axes[LEFT_HORIZONTAL_AXIS]*scale*panMotor.homeVelocity));
				}
				else if (msg->axes[LEFT_HORIZONTAL_AXIS] >= 0){
					sendVelocityMessage(panMotor,(int)(msg->axes[LEFT_HORIZONTAL_AXIS]*scale*panMotor.homeVelocity));
				}
				if(msg->buttons[SELECT_BUTTON] > .1 || msg->buttons[START_BUTTON] ){
					if(msg->buttons[SELECT_BUTTON] > .1){
						sendVelocityMessage(scoopMotor,(int)(-scoopMotor.homeVelocity*scale));
					}
					else sendVelocityMessage(scoopMotor,(int)(scoopMotor.homeVelocity*scale));
				}
				else sendVelocityMessage(scoopMotor,(int)(0));
		}
	else {
		sendVelocityMessage(panMotor,0);
		sendVelocityMessage(shoulderMotor,0);
		sendVelocityMessage(scoopMotor,0);
	}
}

void dropOffRock(){
	shoulderMotor.isHoming=true;
	EposManager::EPOSControl shoulderMsg;
	shoulderMsg.node_id=shoulderMotor.nodeId;
	shoulderMsg.control_mode=EposManager::EPOSControl::ABSOLUTE_POSITION_IMMEDIATE;
	shoulderMsg.setpoint = -230000;
	arm_publisher.publish(shoulderMsg);
//	ros::spinOnce();
	sleep(2);
	EposManager::EPOSControl panMsg;
	panMsg.node_id=panMotor.nodeId;
	panMsg.control_mode=EposManager::EPOSControl::ABSOLUTE_POSITION_IMMEDIATE;
	panMsg.setpoint = 50;
	arm_publisher.publish(panMsg);
//	ros::spinOnce();
	sleep(3);
	EposManager::EPOSControl scoopMsg;
	scoopMsg.node_id=scoopMotor.nodeId;
	scoopMsg.control_mode=EposManager::EPOSControl::ABSOLUTE_POSITION_IMMEDIATE;
	scoopMsg.setpoint = -1300000;
	arm_publisher.publish(scoopMsg);
	shoulderMotor.isHoming=false;
	sleep(4);

}

void sendVelocityMessage(homingData motor, int velocity){
	EposManager::EPOSControl controlMsg;
	controlMsg.node_id=motor.nodeId;
	controlMsg.control_mode=EposManager::EPOSControl::VELOCITY;
	controlMsg.setpoint=velocity;
	arm_publisher.publish(controlMsg);
}

int main (int argc, char **argv){
	ros::init(argc, argv, "ArmManager");
	ros::NodeHandle n;

	panMotor.nodeId=ARM_PAN;
	scoopMotor.nodeId=ARM_SCOOP;
	shoulderMotor.nodeId=ARM_SHOULDER;

	panMotor.timesToIgnore=5;
	scoopMotor.timesToIgnore=5;
	shoulderMotor.timesToIgnore=5;

	panMotor.homeVelocity = 1000;
	scoopMotor.homeVelocity = 1000;
	shoulderMotor.homeVelocity = 1000;

	panMotor.homeCurrent = 100;
	scoopMotor.homeCurrent = 100;
	shoulderMotor.homeCurrent = 100;

	ros::param::get("~Pan_Velocity",panMotor.homeVelocity);
	ros::param::get("~Pan_Current",panMotor.homeCurrent);
	ros::param::get("~Shoulder_Velocity",shoulderMotor.homeVelocity);
	ros::param::get("~Shoulder_Current",shoulderMotor.homeCurrent);
	ros::param::get("~Scoop_Velocity",scoopMotor.homeVelocity);
	ros::param::get("~Scoop_Current",scoopMotor.homeCurrent);

	panMotor.angle=0;
	shoulderMotor.angle=0;

	panMotor.isHoming=false;
	shoulderMotor.isHoming=false;
	scoopMotor.isHoming=false;


	ros::Subscriber joy_subscriber = n.subscribe("joy", 1, joyCallback);
	ros::Subscriber driver_joy_subscriber = n.subscribe("DriverJoy", 1, driverJoyCallback);

//	ros::Subscriber arm_motor_info_subscriber = n.subscribe("motors/Arm_Motors/Motor_Info", 1, motorInfoCallback);
	arm_publisher = n.advertise<EposManager::EPOSControl>("motors/Arm_Motors/Motor_Control", 5);
	group_arm_publisher = n.advertise<EposManager::GroupEPOSControl>("motors/Arm_Motors/Group_Motor_Control", 5);



	ros::spin();
}
