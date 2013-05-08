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

	std::string ObjectLocation("ObjectPose"); ///String containing the topic name for ObjectLocation
	std::string DesiredPosition("DesiredPosition"); ///String containing the topic name for DesiredPosition

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(ObjectLocation, ObjectLocation))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Object Location Topic <%s>!", ObjectLocation.c_str(), ObjectLocation.c_str());
	if (!param_nh.getParam(DesiredPosition, DesiredPosition))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Desired Position Topic <%s>!", DesiredPosition.c_str(), DesiredPosition.c_str());

	ROS_DEBUG("Using Object Location Topic Name: <%s>", ObjectLocation.c_str());
	ROS_DEBUG("Using Desired Position Topic Name: <%s>", DesiredPosition.c_str());

	ROS_INFO("Starting Up Object to Pose...");


	desired_position_pub = nh.advertise<geometry_msgs::PoseStamped>(DesiredPosition, 1,true);
	object_location_sub = nh.subscribe(ObjectLocation, 1, &ObjectLocationMSG);

	ros::spin();
}

