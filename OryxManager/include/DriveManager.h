/*
 * DriveManager.h
 *
 *  Created on: Jan 26, 2012
 *      Author: oryx
 */

#ifndef DRIVEMANAGER_H_
#define DRIVEMANAGER_H_

#include "OryxManager.h"
#include "OryxManager/DriveManagerConfig.h"


double maxVelocity = 1.2;
double baseWidth = 0.39; //distance between left and right wheel midpoints in meters
double baseLength = 0.33; //distance between front and back wheel midpoints in meters
int maxRPM;
ros::Publisher group_drive_publisher;

int velocityToRPM(float velocity);
EposManager::GroupEPOSControl joyToDriveMessage(float left_value, float right_value, float scale);

#endif /* DRIVEMANAGER_H_ */
