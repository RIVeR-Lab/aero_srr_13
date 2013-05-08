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
#include <aero_control/arm_controller.h>

using namespace aero_control;

ArmController::ArmController(ros::NodeHandle nh, std::string ObjectPose,
		std::string ArmPose, std::string SetFingerPosition) {

	this->sub_object_position = nh.subscribe(ObjectPose, 1,
			&ArmController::ObjectPosition, this);
	this->pub_arm_position = nh.advertise<geometry_msgs::PoseStamped>(ArmPose,
			2);


	this->pub_set_finger_position = nh.advertise<jaco_driver::finger_position>(SetFingerPosition,
				2);

}

void ArmController::ObjectPosition(
		const aero_srr_msgs::ObjectLocationMsgConstPtr& object) {
	tf::Matrix3x3 grasp_rpy;
	tf::Quaternion grasp_quaternion;

	geometry_msgs::PoseStamped arm_pose;
jaco_driver::finger_position fingers;


listener.waitForTransform("arm_base", object->pose.header.frame_id, object->pose.header.stamp, ros::Duration(1.0) );
	listener.transformPose("arm_base", object->pose, arm_pose);


	grasp_rpy.setEulerYPR(1.5, -0.7, -1.6);
	grasp_rpy.getRotation(grasp_quaternion);
	ROS_INFO("here");

	arm_pose.pose.position.y -= 0.1;
	float y_temp  = arm_pose.pose.position.y;
	tf::quaternionTFToMsg(grasp_quaternion,arm_pose.pose.orientation);
	arm_pose.pose.position.z = 0.2;
	arm_pose.pose.position.y -= 0.15;
	//arm_pose.pose.position.x= 0.4;

ROS_INFO("Got Point");
for(int x = 0; x<20; x++)
{
	arm_pose.header.stamp = ros::Time().now();

	pub_arm_position.publish(arm_pose);
	ros::Duration(0.5).sleep();

}
	arm_pose.pose.position.z = -0.08;


	ROS_INFO("dropping");
	for(int x = 0; x<30; x++)
	{
	arm_pose.header.stamp = ros::Time().now();
	pub_arm_position.publish(arm_pose);
	ros::Duration(0.5).sleep();

	}
	ros::Duration(2).sleep();

	ROS_INFO("fingers");

	fingers.Finger_1 = 0;
	fingers.Finger_2 = 0;
	fingers.Finger_3 = 0;
	pub_set_finger_position.publish(fingers);

	ros::Duration(5).sleep();


	ROS_INFO("move");

	arm_pose.pose.position.y = y_temp;

	for(int x = 0; x<20; x++)
	{
	arm_pose.header.stamp = ros::Time().now();

	pub_arm_position.publish(arm_pose);
	ros::Duration(0.5).sleep();
	}

	ros::Duration(2).sleep();

	ROS_INFO("grab");

	fingers.Finger_1 = 54;
	fingers.Finger_2 = 54;
	fingers.Finger_3 = 54;

	pub_set_finger_position.publish(fingers);

	ros::Duration(5).sleep();
	ROS_INFO("raise");

	arm_pose.pose.position.z = 0.2;

	for(int x = 0; x<20; x++)
	{
	arm_pose.header.stamp = ros::Time().now();

	pub_arm_position.publish(arm_pose);

	ros::Duration(0.5).sleep();
	}

}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string DesiredPosition("DesiredPosition"); ///String containing the topic name for arm position
	std::string ObjectPose("ObjectPose"); ///String containing the topic name for object position
	std::string SetFingerPosition("SetFingerPosition"); ///String containing the topic name for SetFingerPosition


	if (argc < 1) {
		ROS_INFO(
				"Usage: aero_arm_controller object_position_topic arm_position_topic set_finger_position_topic");
		return 1;
	} else {
		//Grab the topic parameters, print warnings if using default values
		if (!param_nh.getParam(ObjectPose, ObjectPose))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Object Position Topic <%s>!", ObjectPose.c_str(), ObjectPose.c_str());
		if (!param_nh.getParam(DesiredPosition, DesiredPosition))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Arm Position Topic <%s>!", DesiredPosition.c_str(), DesiredPosition.c_str());
		if (!param_nh.getParam(SetFingerPosition, SetFingerPosition))
					ROS_WARN(
							"Parameter <%s> Not Set. Using Default Set Finger Position Topic <%s>!", SetFingerPosition.c_str(), SetFingerPosition.c_str());
	}

//Print out received topics
	ROS_DEBUG("Got Object Position Topic Name: <%s>", ObjectPose.c_str());
	ROS_DEBUG("Got Arm Position Topic Name: <%s>", DesiredPosition.c_str());
	ROS_DEBUG("Got Set Finger Position Topic Name: <%s>", SetFingerPosition.c_str());

	ROS_INFO("Starting Up Arm Controller...");

//create the arm object
	ArmController arm(nh, ObjectPose, DesiredPosition,SetFingerPosition);

	ros::spin();
}

