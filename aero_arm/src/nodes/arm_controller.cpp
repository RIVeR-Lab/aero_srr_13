//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

/**
 * @file jaco_arm_driver.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File
#include <aero_arm/arm_controller.h>

using namespace aero_arm;

Arm_Controller::Arm_Controller(ros::NodeHandle nh, std::string ObjectPose,
		std::string ArmPose) {

	this->sub_object_position = nh.subscribe(ObjectPose, 1,
			&Arm_Controller::ObjectPosition, this);
	this->pub_arm_position = nh.advertise<geometry_msgs::PoseStamped>(ArmPose,
			2);

}

void Arm_Controller::ObjectPosition(
		const geometry_msgs::PoseStampedConstPtr& object_pos) {
	tf::Matrix3x3 grasp_rpy;
	tf::Quaternion grasp_quaternion;

	geometry_msgs::PoseStamped arm_pose;

	listener.transformPose("arm_base", object_pos, arm_pose);

	grasp_rpy.setEulerYPR(3.14, 0, 0);
	grasp_rpy.getRotation(grasp_quaternion);

	arm_pose.pose.orientation = grasp_quaternion;
	arm_pose.pose.position.z +=0.05;

	pub_arm_position.publish(arm_pose);
}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "aero_arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string ArmPose("ArmPose"); ///String containing the topic name for arm position
	std::string ObjectPose("ObjectPose"); ///String containing the topic name for object position

	if (argc < 1) {
		ROS_INFO(
				"Usage: aero_arm_controller object_position_topic arm_position_topic");
		return 1;
	} else {
		//Grab the topic parameters, print warnings if using default values
		if (!param_nh.getParam(ObjectPose, ObjectPose))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Object Position Topic <%s>!", ObjectPose.c_str(), ObjectPose.c_str());
		if (!param_nh.getParam(ArmPose, ArmPose))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Arm Position Topic <%s>!", ArmPose.c_str(), ArmPose.c_str());
	}

//Print out received topics
	ROS_DEBUG("Got Object Position Topic Name: <%s>", ObjectPose.c_str());
	ROS_DEBUG("Got Arm Position Topic Name: <%s>", ArmPose.c_str());

	ROS_INFO("Starting Up Arm Controller...");

//create the arm object
	Arm_Controller arm(nh, ObjectPose, ArmPose);

	ros::spin();
}

