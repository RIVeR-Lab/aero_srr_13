/*
 * LowLevelDriveController.h
 *
 *  Created on: Oct 7, 2012
 *      Author: parallels
 */

#ifndef LOWLEVELDRIVECONTROLLER_H_
#define LOWLEVELDRIVECONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
/**
 * @brief Low level velocity-arc based drive controller
 *
 * Implements a velocity-control based controller which is capable of performing arc based motions. There are seperate
 * control modes for operation based on the capabilities of the base platform:
 * *Without Swerve Capabilities
 *  -Standard Tank Steer Velocity-Arc
 * *With Swerve Capabilities
 *  -Smooth Velocity-Arc
 *  -Translation
 */
class LowLevelDriveController{
public:
	/**
	 * Creates a new LowLevelDriveController with standard initialization parameters
	 * @param drive_velocity_topic		The topic name to publish wheel velocity messages to
	 * @param drive_swerve_topic		The topic name to publish swerve position messages to
	 * @param drive_capabilities_topic	The topic name to poll for DriveManager capabilities
	 * @param baseLength				The length of the base platform from front wheel center to rear wheel center
	 * @param baseWidth					The width of the base platform from left wheel center to right wheel center
	 */
	LowLevelDriveController(std::string drive_velocity_topic,
					std::string drive_swerve_topic,
					std::string drive_capabilities_topic,
					double baseLength,
					double baseWidth);
	///Default constructor
	virtual ~LowLevelDriveController();
private:
	double baseLength;				///The length of the platform
	double baseWidth;				///The width of the platform
	ros::NodeHandle nh;				///Node handle into the ROS system
	ros::Publisher velocity_pub;	///Publisher for sending velocity messages to DriveManager
	ros::Publisher swerve_pub;		///Publisher for sending swerve message to DriveManager
};


#endif /* LOWLEVELDRIVECONTROLLER_H_ */
