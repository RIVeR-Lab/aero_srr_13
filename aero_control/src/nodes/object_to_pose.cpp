//============================================================================
// Name        : Jaco.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

/**
 * @file jaco_arm_control.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>

ros::Subscriber object_location_sub;
ros::Publisher desired_position_pub;

void ObjectLocationMSG(const aero_srr_msgs::ObjectLocationMsgConstPtr& object_location) {
	desired_position_pub.publish(object_location->pose);
}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "object_to_pose");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string object_location("object_location"); ///String containing the topic name for ObjectLocation
	std::string desired_position("desired_position"); ///String containing the topic name for desired_position

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(object_location, object_location))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Object Location Topic <%s>!", object_location.c_str(), object_location.c_str());
	if (!param_nh.getParam(desired_position, desired_position))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Desired Position Topic <%s>!", desired_position.c_str(), desired_position.c_str());

	ROS_DEBUG("Using Object Location Topic Name: <%s>", object_location.c_str());
	ROS_DEBUG("Using Desired Position Topic Name: <%s>", desired_position.c_str());

	ROS_INFO("Starting Up Object to Pose...");


	desired_position_pub = nh.advertise<geometry_msgs::PoseStamped>(desired_position, 1,true);
	object_location_sub = nh.subscribe(object_location, 1, &ObjectLocationMSG);

	ros::spin();
}

