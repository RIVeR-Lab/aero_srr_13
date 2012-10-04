/*
 * ArmManager.h
 *
 *  Created on: May 8, 2012
 *      Author: oryx
 */

#ifndef ARMMANAGER_H_
#define ARMMANAGER_H_
#include "OryxManager.h"

ros::Publisher arm_publisher;
ros::Publisher group_arm_publisher;
ros::Publisher armOrientationPublisher;
void dropOffRock();
long dropOffPosition;
homingData shoulderMotor, scoopMotor, panMotor;
void sendPositionMessage(homingData motor, int setpoint, bool absolute);
void sendVelocityMessage(homingData motor, int velocity);
#endif /* ARMMANAGER_H_ */
