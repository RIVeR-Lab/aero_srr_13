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

ArmController::ArmController(ros::NodeHandle nh, ros::NodeHandle param_nh) {

	std::string ArmPose("DesiredARMPosition"); ///String containing the topic name for arm position
	std::string ObjectPose("ObjectPose"); ///String containing the topic name for object position
	std::string SetFingerPosition("SetFingerPosition"); ///String containing the topic name for SetFingerPosition
	std::string AeroState("aero/supervisor/state"); ///String containing the topic name for AeroState
	std::string AeroStateTransition("aero/supervisor/control_mode"); ///String containing the topic name for AeroStateTransition

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(ObjectPose, ObjectPose))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Object Position Topic <%s>!", ObjectPose.c_str(), ObjectPose.c_str());
	if (!param_nh.getParam(ArmPose, ArmPose))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Arm Position Topic <%s>!", ArmPose.c_str(), ArmPose.c_str());
	if (!param_nh.getParam(SetFingerPosition, SetFingerPosition))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Set Finger Position Topic <%s>!", SetFingerPosition.c_str(), SetFingerPosition.c_str());
	if (!param_nh.getParam(AeroState, AeroState))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Aero State Topic <%s>!", AeroState.c_str(), AeroState.c_str());

	if (!param_nh.getParam(AeroStateTransition, AeroStateTransition))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Aero State Transition Topic <%s>!", AeroStateTransition.c_str(), AeroStateTransition.c_str());

	//Print out received topics
	ROS_DEBUG("Got Object Position Topic Name: <%s>", ObjectPose.c_str());
	ROS_DEBUG("Got Arm Position Topic Name: <%s>", ArmPose.c_str());
	ROS_DEBUG("Got Set Finger Position Topic Name: <%s>", SetFingerPosition.c_str());
	ROS_DEBUG("Using Aero State Topic Name: <%s>", AeroState.c_str());
	ROS_DEBUG("Using Aero State Transition Topic Name: <%s>", AeroStateTransition.c_str());

	ROS_INFO("Starting Up Arm Controller...");

	this->active_state = false;
	this->previous_state = aero_srr_msgs::AeroState::STARTUP;

	/* Messages */
	this->sub_object_position = nh.subscribe(ObjectPose, 1, &ArmController::ObjectPositionMSG,
			this);
	this->aero_state_sub = nh.subscribe(AeroState, 1, &ArmController::AeroStateMSG, this);
	this->pub_arm_position = nh.advertise<geometry_msgs::PoseStamped>(ArmPose, 2);

	this->pub_set_finger_position = nh.advertise<jaco_driver::finger_position>(SetFingerPosition,
			2);
	/* Services */
	this->aero_state_transition_srv_client =
			nh.serviceClient<aero_srr_msgs::StateTransitionRequest>(AeroStateTransition);
}

void ArmController::ObjectPositionMSG(const aero_srr_msgs::ObjectLocationMsgConstPtr& object) {
	geometry_msgs::PoseStamped obj_pose;

	if (active_state == true) {

		try {
			listener.waitForTransform("/arm_base", object->pose.header.frame_id,
					object->pose.header.stamp, ros::Duration(1.0));
			listener.transformPose("/arm_base", object->pose, obj_pose);

//		tf::Matrix3x3 grasp_rpy;
//		tf::Quaternion grasp_quaternion;
//
//		geometry_msgs::PoseStamped arm_pose;
//		jaco_driver::finger_position fingers;
//
//		listener.waitForTransform("arm_base", object->pose.header.frame_id,
//				object->pose.header.stamp, ros::Duration(1.0));
//		listener.transformPose("arm_base", object->pose, arm_pose);
//
//		//grasp_rpy.setEulerYPR(1.5, -0.7, -1.6);
//		//grasp_rpy.getRotation(grasp_quaternion);
//
//
//		ROS_INFO("here");
//
//		arm_pose.pose.position.x -= 0.1;
//		float y_temp = arm_pose.pose.position.y;
//		tf::quaternionTFToMsg(grasp_quaternion, arm_pose.pose.orientation);
//		arm_pose.pose.position.z = 0.2312;
//		arm_pose.pose.position.y -= 0.15;
//		//arm_pose.pose.position.x= 0.4;
//
//		ROS_INFO("Got Point");
//		for (int x = 0; x < 20; x++) {
//			arm_pose.header.stamp = ros::Time().now();
//
//			pub_arm_position.publish(arm_pose);
//			ros::Duration(0.5).sleep();
//
//		}
//		arm_pose.pose.position.z = -0.08;
//
//		ROS_INFO("dropping");
//		for (int x = 0; x < 30; x++) {
//			arm_pose.header.stamp = ros::Time().now();
//			pub_arm_position.publish(arm_pose);
//			ros::Duration(0.5).sleep();
//
//		}
//		ros::Duration(2).sleep();
//
//		ROS_INFO("fingers");
//
//		fingers.Finger_1 = 0;
//		fingers.Finger_2 = 0;
//		fingers.Finger_3 = 0;
//		pub_set_finger_position.publish(fingers);
//
//		ros::Duration(5).sleep();
//
//		ROS_INFO("move");
//
//		arm_pose.pose.position.y = y_temp;
//
//		for (int x = 0; x < 20; x++) {
//			arm_pose.header.stamp = ros::Time().now();
//
//			pub_arm_position.publish(arm_pose);
//			ros::Duration(0.5).sleep();
//		}
//
//		ros::Duration(2).sleep();
//
//		ROS_INFO("grab");
//
//		fingers.Finger_1 = 54;
//		fingers.Finger_2 = 54;
//		fingers.Finger_3 = 54;
//
//		pub_set_finger_position.publish(fingers);
//
//		ros::Duration(5).sleep();
//		ROS_INFO("raise");
//
//		arm_pose.pose.position.z = 0.2;
//
//		for (int x = 0; x < 20; x++) {
//			arm_pose.header.stamp = ros::Time().now();
//
//			pub_arm_position.publish(arm_pose);
//
//			ros::Duration(0.5).sleep();
//		}

			jaco_driver::finger_position fingers;

			geometry_msgs::PoseStamped arm_pose;

			arm_pose.pose.position.x = 0.14648;

			arm_pose.pose.position.y = -0.47118;
			arm_pose.pose.position.z = 0.2312;

			arm_pose.pose.orientation.x = 0.717179;
			arm_pose.pose.orientation.y = 0.02939;
			arm_pose.pose.orientation.z = 0.11574;
			arm_pose.pose.orientation.w = -0.6865;

			arm_pose.header.frame_id = "/jaco_api_origin";
			arm_pose.header.stamp = ros::Time().now();

			for (int x = 0; x < 20; x++) {
				arm_pose.header.stamp = ros::Time().now();

				pub_arm_position.publish(arm_pose);
				ros::Duration(0.5).sleep();

			}
			ROS_INFO("fingers");

			fingers.Finger_1 = 0;
			fingers.Finger_2 = 0;
			fingers.Finger_3 = 0;
			pub_set_finger_position.publish(fingers);

			ros::Duration(5).sleep();
			arm_pose.pose.position.x = 0.14648;

			arm_pose.pose.position.y = -0.47118;
			arm_pose.pose.position.z = -0.1;

			arm_pose.pose.orientation.x = 0.717179;
			arm_pose.pose.orientation.y = 0.02939;
			arm_pose.pose.orientation.z = 0.11574;
			arm_pose.pose.orientation.w = -0.6865;

			arm_pose.header.frame_id = "/jaco_api_origin";
			arm_pose.header.stamp = ros::Time().now();

			for (int x = 0; x < 20; x++) {
				arm_pose.header.stamp = ros::Time().now();

				pub_arm_position.publish(arm_pose);
				ros::Duration(0.5).sleep();

			}
			obj_pose.header.stamp = ros::Time().now();

			obj_pose.pose.orientation.x = 0.717179;
			obj_pose.pose.orientation.y = 0.02939;
			obj_pose.pose.orientation.z = 0.11574;
			obj_pose.pose.orientation.w = -0.6865;
			for (int x = 0; x < 20; x++) {
				arm_pose.header.stamp = ros::Time().now();

				pub_arm_position.publish(obj_pose);
				ros::Duration(0.5).sleep();

			}

			ROS_INFO("fingers");

			fingers.Finger_1 = 54;
			fingers.Finger_2 = 54;
			fingers.Finger_3 = 54;
			pub_set_finger_position.publish(fingers);

			ros::Duration(5).sleep();

			obj_pose.pose.position.z = 0.2312;

			for (int x = 0; x < 20; x++) {
				obj_pose.header.stamp = ros::Time().now();

				pub_arm_position.publish(obj_pose);
				ros::Duration(0.5).sleep();

			}

			obj_pose.pose.position.x = 0.14648;

			obj_pose.pose.position.y = -0.47118;
			obj_pose.pose.position.z = 0.2312;

			obj_pose.pose.orientation.x = 0.717179;
			obj_pose.pose.orientation.y = 0.02939;
			obj_pose.pose.orientation.z = 0.11574;
			obj_pose.pose.orientation.w = -0.6865;

			obj_pose.header.frame_id = "/jaco_api_origin";
			obj_pose.header.stamp = ros::Time().now();

			for (int x = 0; x < 20; x++) {
				arm_pose.header.stamp = ros::Time().now();

				pub_arm_position.publish(obj_pose);
				ros::Duration(0.5).sleep();

			}
			aero_srr_msgs::StateTransitionRequest state_transition;

			state_transition.request.requested_state.state = previous_state;
			state_transition.request.requested_state.header.stamp = ros::Time().now();
			aero_state_transition_srv_client.call(state_transition);
			this->active_state = false;

		} catch (std::exception& e) {
			ROS_ERROR_STREAM_THROTTLE(1, e.what());
		}

	}
}

void ArmController::AeroStateMSG(const aero_srr_msgs::AeroState& aero_state) {

	switch (aero_state.state) {

	case aero_srr_msgs::AeroState::PICKUP:
		this->active_state = true;
		break;
	case aero_srr_msgs::AeroState::COLLECT:
		this->active_state = false;
		break;
	case aero_srr_msgs::AeroState::SHUTDOWN:
		this->active_state = false;
		ros::shutdown();
		break;
	case aero_srr_msgs::AeroState::PAUSE:
		this->active_state = false;
		break;
	case aero_srr_msgs::AeroState::ERROR: //TODO Does this node need to do anything on error?
	default:
		this->active_state = false;
		previous_state = aero_state.state;

		break;
	}
}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

//create the arm object
	ArmController arm(nh, param_nh);

	ros::spin();
}

