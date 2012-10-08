/*
 * LowLevelDriveController.cpp
 *
 *  Created on: Oct 7, 2012
 *      Author: parallels
 */

#include "LowLevelDriveController.h"

LowLevelDriveController::LowLevelDriveController(std::string drive_velocity_topic,
												 std::string drive_swerve_topic,
												 std::string drive_capabilities_topic,
												 double baseLength,
												 double baseWidth) {
	ROS_INFO("Starting Up Low Level Drive Controller");
	//TODO actually set up correct message publishing data
	//Set up publisher to the DriveManager wheel velocity topic
	this->velocity_pub = nh.advertise<std_msgs::String>(drive_velocity_topic.c_str(),2);
	//Set up publisher to the DriveManager swerve control topic
	this->swerve_pub = nh.advertise<std_msgs::String>(drive_swerve_topic.c_str(),2);
	//set up dimensional parameters
	this->baseLength = baseLength;
	this->baseWidth  = baseWidth;
	ROS_DEBUG("Got a platform size of <L=%f,W=%f>",baseLength, baseWidth);
}

LowLevelDriveController::~LowLevelDriveController() {
	// TODO Auto-generated destructor stub
}
