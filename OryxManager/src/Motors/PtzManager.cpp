/*
 * PtzManager.cpp
 *
 *  Created on: Jan 26, 2012
 *      Author: oryx
 */

#include "PtzManager.h"

#define topSpeed 2000

/**
 * This is the callback for PTZ control in single user mode. The driver needs to press
 * the left bumper button to control the camera.
 */
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
	//If the Left Button is Pressed
	if (msg->buttons[LEFT_BUTTON] >.1) {
		//If the start and select buttons are pressed, enable boom control
		if(msg->buttons[SELECT_BUTTON]>0 && msg->buttons[START_BUTTON]>0){
			if (msg->axes[D_PAD_UP_DOWN] > 0)
				sendVelocityMessage(boomMotor,3000);
			else if (msg->axes[D_PAD_UP_DOWN] < 0)
				sendVelocityMessage(boomMotor,-3000);
			else sendVelocityMessage(boomMotor,0);
		}
		//Joystick DPAD will control stepping of motors
		else if(msg->axes[D_PAD_UP_DOWN] != 0 || msg->axes[D_PAD_LEFT_RIGHT] !=0){
			if (msg->axes[D_PAD_UP_DOWN] > 0)
				sendPositionMessage(tiltMotor, -upDownStep, false);
			else if (msg->axes[D_PAD_UP_DOWN] < 0)
				sendPositionMessage(tiltMotor, upDownStep, false);
			if (msg->axes[D_PAD_LEFT_RIGHT] > 0)
				sendPositionMessage(panMotor, -leftRightStep, false);
			else if (msg->axes[D_PAD_LEFT_RIGHT] < 0)
				sendPositionMessage(panMotor, leftRightStep, false);
		}
		//Control velocities with joysticks
		else{
			if (msg->axes[LEFT_VERTICAL_AXIS] < 0){
				if(abs(tiltMotor.currentPosition - tiltMotor.maxValue)
						< abs(tiltMotor.homePos-tiltMotor.maxValue)*.1)
					 sendVelocityMessage(tiltMotor,0);
				else sendVelocityMessage(tiltMotor,(int)(msg->axes[LEFT_VERTICAL_AXIS]*-topSpeed));
			}
			else if (msg->axes[LEFT_VERTICAL_AXIS] >= 0){
				if(abs(tiltMotor.currentPosition - tiltMotor.minValue) <
						abs(tiltMotor.homePos-tiltMotor.minValue)*.1)
					sendVelocityMessage(tiltMotor,0);
				else sendVelocityMessage(tiltMotor,(int)(msg->axes[LEFT_VERTICAL_AXIS]*-topSpeed));
			}
			if (msg->axes[LEFT_HORIZONTAL_AXIS] < 0){
				if(abs(panMotor.currentPosition - panMotor.maxValue) <
						abs(panMotor.homePos-panMotor.maxValue)*.1)
					sendVelocityMessage(panMotor,0);
				else sendVelocityMessage(panMotor,(int)(msg->axes[LEFT_HORIZONTAL_AXIS]*-topSpeed));
			}
			else if (msg->axes[LEFT_HORIZONTAL_AXIS] >= 0){
				if(abs(panMotor.currentPosition - panMotor.minValue) <
						abs(panMotor.homePos-panMotor.minValue)*.1)
					sendVelocityMessage(panMotor,0);
				else sendVelocityMessage(panMotor,(int)(msg->axes[LEFT_HORIZONTAL_AXIS]*-topSpeed));
			}

		}
	}
	//Tell pan-tilt to stop if joystick message not meant for them
	else{
		if(panMotor.isHoming || tiltMotor.isHoming) return;
		sendVelocityMessage(panMotor,0);
		sendVelocityMessage(tiltMotor,0);
	}
}

/**
 * This is the callback for PTZ control in multi user mode. ANy joystick command will automatically
 * be treated as though it were meant for the PTZ
 */
void ptzJoyCallback(const sensor_msgs::Joy::ConstPtr& msg){
		//If the start and select buttons are pressed, enable boom control
		if(msg->buttons[SELECT_BUTTON]>0 && msg->buttons[START_BUTTON]>0){
			if (msg->axes[D_PAD_UP_DOWN] > 0)
				sendVelocityMessage(boomMotor,3000);
			else if (msg->axes[D_PAD_UP_DOWN] < 0)
				sendVelocityMessage(boomMotor,-3000);
			else sendVelocityMessage(boomMotor,0);
		}
		//Joystick DPAD will control stepping of motors
		else if(msg->axes[D_PAD_UP_DOWN] != 0 || msg->axes[D_PAD_LEFT_RIGHT] !=0){
			if (msg->axes[D_PAD_UP_DOWN] > 0)
				sendPositionMessage(tiltMotor, -upDownStep, false);
			else if (msg->axes[D_PAD_UP_DOWN] < 0)
				sendPositionMessage(tiltMotor, upDownStep, false);
			if (msg->axes[D_PAD_LEFT_RIGHT] > 0)
				sendPositionMessage(panMotor, -leftRightStep, false);
			else if (msg->axes[D_PAD_LEFT_RIGHT] < 0)
				sendPositionMessage(panMotor, leftRightStep, false);
		}
		//Control velocities with joysticks
		else{
			if (msg->axes[LEFT_VERTICAL_AXIS] < 0){
				if(abs(tiltMotor.currentPosition - tiltMotor.maxValue)
						< abs(tiltMotor.homePos-tiltMotor.maxValue)*.1)
					 sendVelocityMessage(tiltMotor,0);
				else sendVelocityMessage(tiltMotor,(int)(msg->axes[LEFT_VERTICAL_AXIS]*-topSpeed));
			}
			else if (msg->axes[LEFT_VERTICAL_AXIS] >= 0){
				if(abs(tiltMotor.currentPosition - tiltMotor.minValue) <
						abs(tiltMotor.homePos-tiltMotor.minValue)*.1)
					sendVelocityMessage(tiltMotor,0);
				else sendVelocityMessage(tiltMotor,(int)(msg->axes[LEFT_VERTICAL_AXIS]*-topSpeed));
			}
			if (msg->axes[LEFT_HORIZONTAL_AXIS] < 0){
				if(abs(panMotor.currentPosition - panMotor.maxValue) <
						abs(panMotor.homePos-panMotor.maxValue)*.1)
					sendVelocityMessage(panMotor,0);
				else sendVelocityMessage(panMotor,(int)(msg->axes[LEFT_HORIZONTAL_AXIS]*-topSpeed));
			}
			else if (msg->axes[LEFT_HORIZONTAL_AXIS] >= 0){
				if(abs(panMotor.currentPosition - panMotor.minValue) <
						abs(panMotor.homePos-panMotor.minValue)*.1)
					sendVelocityMessage(panMotor,0);
				else sendVelocityMessage(panMotor,(int)(msg->axes[LEFT_HORIZONTAL_AXIS]*-topSpeed));
			}

		}
}


/**
 * The callback for reconfiguring the different manager parameters/functions.
 */
void reconfigureCallback(OryxManager::PTZManagerConfig &config, uint32_t level){
	if(config.Home_Pan == true && previousConfig.Home_Pan == false){
		panMotor.isHoming=true;
		panMotor.maxValue=0;
		panMotor.minValue=0;
		sendVelocityMessage(panMotor,panMotor.homeVelocity);
	}
	if(config.Home_Tilt == true && previousConfig.Home_Tilt == false){
		tiltMotor.isHoming=true;
		tiltMotor.maxValue=0;
		tiltMotor.minValue=0;
		sendVelocityMessage(tiltMotor,tiltMotor.homeVelocity);
	}
	if(config.Home_Boom == true && previousConfig.Home_Boom == false){
		boomMotor.isHoming=true;
		boomMotor.maxValue=0;
		boomMotor.minValue=0;
		if(config.Home_Safely == true)sendVelocityMessage(boomMotor,boomMotor.homeVelocity);
		else {
			driveBoomWithoutMonitoring = true;
			sendVelocityMessage(boomMotor,9000);
		}
	}
	if(config.Home_Pan == false) panMotor.isHoming = false;
	if(config.Home_Tilt == false) tiltMotor.isHoming = false;
	if(config.Home_Boom == false){
		sendVelocityMessage(boomMotor,0);
		boomMotor.isHoming = false;
	}
	previousConfig=config;
	//TODO: Enable control through the set angle sliders
}

/*
 * This publishes the angle of the camera. Right now, it incorrectly uses a quaternion message.
 * Right now, it is in degrees of rotation about the y and z axes. This is because we had issues
 * converting between degrees and quaternions. In the future, this should still be sent as a
 * quaternion, but should actually use the quaternion format.
 */
void publishCameraAngle(){
	//TODO: Use actual quaternion format
	geometry_msgs::Quaternion msg;
	msg.z=panMotor.angle;
	msg.y=tiltMotor.angle;
	cameraOrientationPublisher.publish(msg);
}

/**
 * Callback to be run whenever a new motor info message is received
 */
void motorInfoCallback(const EposManager::MotorInfo::ConstPtr& msg){
	if(msg->node_id == panMotor.nodeId){
		//Get Position of motor
		panMotor.currentPosition=msg->motor_position;
		//If motor has discovered boundary points, and is in between them, allow user control
		if(panMotor.maxValue!=0 && panMotor.minValue!=0 && !isMotorTooClose(panMotor))
			panMotor.isReadyForControl=true;
		//If it is homing mode, home it
		if(panMotor.isHoming) homeMotor(&panMotor, msg);
		//If motor is no longer homing and is too close stop it
		else if(panMotor.isReadyForControl && isMotorTooClose(panMotor) ){
			sendVelocityMessage(panMotor,0);
		}
		//Calculate and publish the camera angle
		panMotor.angle =((double)(panMotor.currentPosition - panMotor.homePos))
				/PAN_MOTOR_GEAR_RATIO/512*90;
		publishCameraAngle();

	}
	if(msg->node_id == tiltMotor.nodeId ){
		tiltMotor.currentPosition = msg->motor_position;
		if(tiltMotor.maxValue!=0 && tiltMotor.minValue!=0 && !isMotorTooClose(tiltMotor))
			tiltMotor.isReadyForControl=true;
		if (tiltMotor.isHoming) homeMotor(&tiltMotor, msg);
		else if(tiltMotor.isReadyForControl && isMotorTooClose(tiltMotor)){
			sendVelocityMessage(tiltMotor,0);
		}
		//Calculate and publish the camera angle
		tiltMotor.angle =((double)(tiltMotor.currentPosition - tiltMotor.homePos))
				/TILT_MOTOR_GEAR_RATIO/512*90;
	}
	if(msg->node_id == boomMotor.nodeId ){
		boomMotor.currentPosition=msg->motor_position;
		if(boomMotor.maxValue!=0 && boomMotor.minValue!=0 && !isMotorTooClose(boomMotor))
			boomMotor.isReadyForControl=true;
		if (boomMotor.isHoming){
			homeMotor(&boomMotor, msg);
		}
		else if(boomMotor.isReadyForControl && isMotorTooClose(boomMotor)){
			sendVelocityMessage(boomMotor,0);
		}
	}
}

/**
 * Checks if the motor is too close to a boundary point (hard stop)
 */
bool isMotorTooClose(homingData motor){
	int maxDifference= abs(motor.homePos-motor.maxValue);
	int minDifference= abs(motor.homePos-motor.minValue);
	if( (abs(motor.currentPosition - motor.maxValue) < maxDifference*.1) ||
			(abs(motor.currentPosition - motor.minValue) < minDifference*.1))
			return true;
	return false;
}

/**
 * Sends a velocity message to the specified motor
 */
void sendVelocityMessage(homingData motor, int velocity){

	EposManager::EPOSControl controlMsg;
	controlMsg.node_id=motor.nodeId;
	controlMsg.control_mode=EposManager::EPOSControl::VELOCITY;
	controlMsg.setpoint=velocity;
	ptz_publisher.publish(controlMsg);
}

/**
 * Sends a position message to the specified motor.
 */
void sendPositionMessage(homingData motor, int setpoint, bool absolute){
	if(absolute){
		if (setpoint < motor.minValue) setpoint = motor.minValue;
		else if (setpoint > motor.maxValue) setpoint=motor.maxValue;
	}
	else{
		if(motor.currentPosition+setpoint < motor.minValue) setpoint=0;
		else if(motor.currentPosition+setpoint > motor.maxValue) setpoint=0;
	}
	EposManager::EPOSControl controlMsg;
	controlMsg.node_id=motor.nodeId;
	if(absolute)controlMsg.control_mode=EposManager::EPOSControl::ABSOLUTE_POSITION_IMMEDIATE;
	else controlMsg.control_mode=EposManager::EPOSControl::RELATIVE_POSITION_IMMEDIATE;
	controlMsg.setpoint=setpoint;
	ptz_publisher.publish(controlMsg);
}

/**
 * This function will home the given motor by monitoring current spikes.
 */
void homeMotor(homingData* motor, const EposManager::MotorInfo::ConstPtr& msg){
		//Ignore first few readings for a Motor, as they tend to spike
		motor->isReadyForControl=false;
		if(++motor->ignoreCounter >= motor->timesToIgnore){
			//If the first limit has not been set and hard stop reached
			if(motor->nodeId==PTZ_BOOM && driveBoomWithoutMonitoring){
				if(motor->ignoreCounter == motor->timesToIgnore+75){
					EposManager::EPOSControl controlMsg;
					controlMsg.node_id=motor->nodeId;
					controlMsg.control_mode=EposManager::EPOSControl::VELOCITY;
					controlMsg.setpoint=motor->homeVelocity;
					motor->ignoreCounter =0;
					driveBoomWithoutMonitoring = false;
					ptz_publisher.publish(controlMsg);
					return;
				}
				else return;
			}
			if(abs(msg->motor_current) > motor->homeCurrent && motor->maxValue==0){
				//Generate Motor Message
				EposManager::EPOSControl controlMsg;
				controlMsg.node_id=motor->nodeId;
				controlMsg.control_mode=EposManager::EPOSControl::VELOCITY;
				controlMsg.setpoint=-motor->homeVelocity;
				if(motor->nodeId == PTZ_BOOM){
					controlMsg.setpoint=0;
					motor->minValue=(long int)(msg->motor_position);
					motor->isHoming=false;
					driveBoomWithoutMonitoring=false;
				}
				ptz_publisher.publish(controlMsg);
				motor->maxValue=(long int)(msg->motor_position);
				motor->ignoreCounter=0;
			}
			//If the second limit has not been set and hard stop reached
			else if (abs(msg->motor_current) > motor->homeCurrent && motor->minValue==0){
				EposManager::EPOSControl controlMsg;
				motor->minValue=(long int)(msg->motor_position);
				motor->homePos=(motor->maxValue+motor->minValue)/2;
				controlMsg.node_id=motor->nodeId;
				controlMsg.control_mode=EposManager::EPOSControl::ABSOLUTE_POSITION;
				controlMsg.setpoint=motor->homePos;
				ptz_publisher.publish(controlMsg);
				motor->isHoming=false;

			}
		}
}




int main (int argc, char **argv){
	ros::init(argc, argv, "PTZManager");
	ros::NodeHandle n;
	upDownStep=1000;
	leftRightStep=1000;

	panMotor.nodeId=PTZ_PAN;
	tiltMotor.nodeId=PTZ_TILT;
	boomMotor.nodeId=PTZ_BOOM;

	panMotor.timesToIgnore=5;
	tiltMotor.timesToIgnore=5;
	boomMotor.timesToIgnore=5;

	panMotor.homeVelocity = 1000;
	tiltMotor.homeVelocity = 1000;
	boomMotor.homeVelocity = 1000;

	panMotor.homeCurrent = 100;
	tiltMotor.homeCurrent = 100;
	boomMotor.homeCurrent = 100;

	panMotor.angle=0;
	tiltMotor.angle=0;

	ros::param::get("~Pan_Velocity",panMotor.homeVelocity);
	ros::param::get("~Pan_Current",panMotor.homeCurrent);
	ros::param::get("~Tilt_Velocity",tiltMotor.homeVelocity);
	ros::param::get("~Tilt_Current",tiltMotor.homeCurrent);
	ros::param::get("~Boom_Velocity",boomMotor.homeVelocity);
	ros::param::get("~Boom_Current",boomMotor.homeCurrent);
	ros::param::get("~Up_Down_Step",upDownStep);
	ros::param::get("~Left_Right_Step",leftRightStep);

	panMotor.isHoming=false;
	boomMotor.isHoming=false;
	tiltMotor.isHoming=false;



	ros::Subscriber joy_subscriber = n.subscribe("joy", 1, joyCallback);
	ros::Subscriber ptz_joy_subscriber = n.subscribe("OperatorJoy", 1, ptzJoyCallback);
	ros::Subscriber ptz_motor_info_subscriber = n.subscribe("motors/PTZ_Motors/Motor_Info", 1, motorInfoCallback);

	ptz_publisher = n.advertise<EposManager::EPOSControl>("motors/PTZ_Motors/Motor_Control", 5);
	group_ptz_publisher = n.advertise<EposManager::GroupEPOSControl>("motors/PTZ_Motors/Group_Motor_Control", 5);
	cameraOrientationPublisher = n.advertise<geometry_msgs::Quaternion>("PTZ/Orientation",1);

	dynamic_reconfigure::Server<OryxManager::PTZManagerConfig> server;
	dynamic_reconfigure::Server<OryxManager::PTZManagerConfig>::CallbackType f;

	f = boost::bind(reconfigureCallback, _1 ,_2);
	server.setCallback(f);

	ros::spin();
}



