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

ArmController::ArmController(ros::NodeHandle nh, ros::NodeHandle param_nh)
{

	std::string desired_arm_pose("desired_arm_pose"); ///String containing the topic name for arm position
	std::string arm_state("arm_state"); ///String containing the topic name for arm State
	std::string object_pose("object_pose"); ///String containing the topic name for object position
	std::string set_finger_position("set_finger_position"); ///String containing the topic name for set_finger_position
	std::string aero_state("aero/supervisor/state"); ///String containing the topic name for aero_state
	std::string aero_state_transition("aero/supervisor/control_mode"); ///String containing the topic name for aero_state_transition

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(object_pose, object_pose))
		ROS_WARN("Parameter <%s> Not Set. Using Default Object Position Topic <%s>!", object_pose.c_str(),
				object_pose.c_str());
	if (!param_nh.getParam(desired_arm_pose, desired_arm_pose))
		ROS_WARN("Parameter <%s> Not Set. Using Default Arm Position Topic <%s>!", desired_arm_pose.c_str(),
				desired_arm_pose.c_str());
	if (!param_nh.getParam(set_finger_position, set_finger_position))
		ROS_WARN("Parameter <%s> Not Set. Using Default Set Finger Position Topic <%s>!",
				set_finger_position.c_str(), set_finger_position.c_str());
	if (!param_nh.getParam(aero_state, aero_state))
		ROS_WARN("Parameter <%s> Not Set. Using Default Aero State Topic <%s>!", aero_state.c_str(),
				aero_state.c_str());

	if (!param_nh.getParam(aero_state_transition, aero_state_transition))
		ROS_WARN("Parameter <%s> Not Set. Using Default Aero State Transition Topic <%s>!",
				aero_state_transition.c_str(), aero_state_transition.c_str());

	if (!param_nh.getParam(arm_state, arm_state))
		ROS_WARN("Parameter <%s> Not Set. Using Default Arm State Topic <%s>!", arm_state.c_str(),
				arm_state.c_str());
	//Print out received topics
	ROS_DEBUG("Using Object Position Topic Name: <%s>", object_pose.c_str());
	ROS_DEBUG("Using Arm Position Topic Name: <%s>", desired_arm_pose.c_str());
	ROS_DEBUG("Using Set Finger Position Topic Name: <%s>", set_finger_position.c_str());
	ROS_DEBUG("Using Aero State Topic Name: <%s>", aero_state.c_str());
	ROS_DEBUG("Using Aero State Transition Topic Name: <%s>", aero_state_transition.c_str());
	ROS_DEBUG("Using Arm State Topic Name: <%s>", arm_state.c_str());

	ROS_INFO("Starting Up Arm Controller...");

	this->active_state = false;
	this->active_state = true; //need to remove

	this->previous_state = aero_srr_msgs::AeroState::STARTUP;
	this->arm_moving = false;
	this->arm_goal_reached = false;
	this->path_active = false;
	this->path_step_start = true;
	this->path_step_start_time = ros::Time().now();
	/* Messages */
	this->object_position_sub = nh.subscribe(object_pose, 1, &ArmController::ObjectPositionMSG, this);
	//this->aero_state_sub = nh.subscribe(aero_state, 1, &ArmController::AeroStateMSG, this);

	this->arm_state_sub = nh.subscribe(arm_state, 1, &ArmController::ArmStateMSG, this);

	this->arm_position_pub = nh.advertise<geometry_msgs::PoseStamped>(desired_arm_pose, 2);

	this->set_finger_position_pub = nh.advertise<jaco_driver::finger_position>(set_finger_position, 2);

	/* Services */
	this->aero_state_transition_srv_client = nh.serviceClient<aero_srr_msgs::StateTransitionRequest>(
			aero_state_transition);

	/* Timers */
	this->path_timer = nh.createTimer(ros::Duration(0.05), &ArmController::PathTimerCallback, this);
	path_timer.stop();

	this->path_step_num = 0;
	PlanHorizontalPath();

}

void ArmController::PathTimerCallback(const ros::TimerEvent&)
{
	ROS_INFO("Path");
	if (this->path_active == true)
	{
		ROS_INFO("Active");

		ROS_INFO("path_step_num=%d", path_step_num);
		ROS_INFO("path_steps=%d", path_steps);

		ROS_INFO("arm_goal_reached=%d", arm_goal_reached);
		ROS_INFO("path_step_start=%d", path_step_start);

		if (this->path_step_num < this->path_steps)
		{



			if (this->path_step_start == true)
			{
				ROS_INFO("reset time");
				this->path_step_start_time = ros::Time().now();
			}

			if ((arm_goal_reached != true|| this->path_step_start == true) && (ros::Time().now().toSec() - this->path_step_start_time.toSec()) < 20)
			{

				this->path_step_start = false;
				if (arm_path[path_step_num].arm_motion == true)
				{
					ROS_INFO("arm_path");

					geometry_msgs::PoseStamped arm_pose;

					arm_pose.pose.position.x = arm_path[path_step_num].x_pos;
					arm_pose.pose.position.y = arm_path[path_step_num].y_pos;
					arm_pose.pose.position.z = arm_path[path_step_num].z_pos;

					tf::Quaternion q;

					q.setRPY(arm_path[path_step_num].roll_pos, arm_path[path_step_num].pitch_pos,
							arm_path[path_step_num].yaw_pos);

					tf::quaternionTFToMsg(q, arm_pose.pose.orientation);

					arm_pose.header.frame_id = "/arm_mount";

					arm_pose.header.stamp = ros::Time().now();

					arm_position_pub.publish(arm_pose);

				} else if (arm_path[path_step_num].finger_motion == true)
				{
					ROS_INFO("finger_path");

					jaco_driver::finger_position fingers;

					fingers.Finger_1 = arm_path[path_step_num].finger_1_pos;
					fingers.Finger_2 = arm_path[path_step_num].finger_2_pos;
					fingers.Finger_3 = arm_path[path_step_num].finger_3_pos;
					set_finger_position_pub.publish(fingers);
				}
			} else
			{
				ROS_INFO("Next Step");
				this->path_step_num++;
				this->path_step_start = true;
				this->arm_goal_reached = false;
			}

		} else
		{

			path_timer.stop(); //TODO Remove
			ROS_INFO("DONE");

			this->path_active = false;
			this->active_state = false;
			aero_srr_msgs::StateTransitionRequest state_transition;

			state_transition.request.requested_state.state = previous_state;
			state_transition.request.requested_state.header.stamp = ros::Time().now();
			aero_state_transition_srv_client.call(state_transition);
		}

	}

}

void ArmController::ObjectPositionMSG(const aero_srr_msgs::ObjectLocationMsgConstPtr& object)
{
	if (active_state == true)
	{
		ROS_INFO("Active MSG");

		try
		{
			if (path_active == false)
			{

				ROS_INFO("Start");
				try
				{
					listener.waitForTransform("/arm_mount", object->pose.header.frame_id,
							object->pose.header.stamp, ros::Duration(1.0));
					listener.transformPose("/arm_mount", object->pose, this->obj_pose);

					this->object_type = object->object_type;

					path_active = true;
					this->path_step_start = true;
					this->path_step_num = 0;
					PlanHorizontalPath();
					path_timer.start();
				} catch (std::exception& e)
				{
					ROS_ERROR_STREAM_THROTTLE(1, e.what());
				}
			} else
			{
				geometry_msgs::PoseStamped new_obj_pose;
				try
				{
					listener.waitForTransform("/arm_mount", object->pose.header.frame_id,
							object->pose.header.stamp, ros::Duration(1.0));
					listener.transformPose("/arm_mount", object->pose, new_obj_pose);
				} catch (std::exception& e)
				{
					ROS_ERROR_STREAM_THROTTLE(1, e.what());
				}
				//TODO add conditional abort if point moves too far
			}

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
//			arm_position_pub.publish(arm_pose);
//
//			ros::Duration(0.5).sleep();
//		}

//			jaco_driver::finger_position fingers;
//
//			geometry_msgs::PoseStamped arm_pose;
//
//			arm_pose.pose.position.x = 0.25;
//
//			arm_pose.pose.position.y = -0.41;
//			arm_pose.pose.position.z = 0.2312;
//
//			arm_pose.pose.orientation.x = 0.717179;
//			arm_pose.pose.orientation.y = 0.02939;
//			arm_pose.pose.orientation.z = 0.11574;
//			arm_pose.pose.orientation.w = -0.6865;
//
//			arm_pose.header.frame_id = "/jaco_api_origin";
//			arm_pose.header.stamp = ros::Time().now();
//
//			for (int x = 0; x < 20; x++) {
//				arm_pose.header.stamp = ros::Time().now();
//
//				arm_position_pub.publish(arm_pose);
//				ros::Duration(0.5).sleep();
//
//			}
//
//			ros::Duration(2).sleep();
//			ROS_INFO("fingers");
//
//			fingers.Finger_1 = 0;
//			fingers.Finger_2 = 0;
//			fingers.Finger_3 = 0;
//			set_finger_position_pub.publish(fingers);
//
//			ros::Duration(5).sleep();
//
//			arm_pose.pose.position.z = -0.1;
//
//
//			arm_pose.header.frame_id = "/jaco_api_origin";
//			arm_pose.header.stamp = ros::Time().now();
//
//			for (int x = 0; x < 20; x++) {
//				arm_pose.header.stamp = ros::Time().now();
//
//				arm_position_pub.publish(arm_pose);
//				ros::Duration(0.5).sleep();
//
//			}
//			arm_pose.header.stamp = ros::Time().now();
//
//			arm_pose.pose.position.x = 0.35;
//
//			arm_pose.pose.orientation.x = 0.717179;
//			arm_pose.pose.orientation.y = 0.02939;
//			arm_pose.pose.orientation.z = 0.11574;
//			arm_pose.pose.orientation.w = -0.6865;
//			for (int x = 0; x < 20; x++) {
//				arm_pose.header.stamp = ros::Time().now();
//
//				arm_position_pub.publish(arm_pose);
//				ros::Duration(0.5).sleep();
//
//			}
//			ros::Duration(2).sleep();
//			ROS_INFO("fingers");
//
//			fingers.Finger_1 = 54;
//			fingers.Finger_2 = 54;
//			fingers.Finger_3 = 54;
//			set_finger_position_pub.publish(fingers);
//
//			ros::Duration(5).sleep();
//
//			arm_pose.pose.position.z = 0.2312;
//
//			for (int x = 0; x < 20; x++) {
//				arm_pose.header.stamp = ros::Time().now();
//
//				arm_position_pub.publish(arm_pose);
//				ros::Duration(0.5).sleep();
//
//			}
//
//			arm_pose.pose.position.x = 0.3;
//
//			arm_pose.pose.position.y = -0.41;
//			arm_pose.pose.position.z = 0.2312;
//
//			arm_pose.pose.orientation.x = 0.717179;
//			arm_pose.pose.orientation.y = 0.02939;
//			arm_pose.pose.orientation.z = 0.11574;
//			arm_pose.pose.orientation.w = -0.6865;
//
//			arm_pose.header.frame_id = "/jaco_api_origin";
//			arm_pose.header.stamp = ros::Time().now();
//
//			for (int x = 0; x < 20; x++) {
//				arm_pose.header.stamp = ros::Time().now();
//
//				arm_position_pub.publish(arm_pose);
//				ros::Duration(0.5).sleep();
//
//			}
//
//

		} catch (std::exception& e)
		{
			ROS_ERROR_STREAM_THROTTLE(1, e.what());
		}

	}
}

void ArmController::ArmStateMSG(const aero_control::arm_stateConstPtr& arm_state)
{
	arm_moving = arm_state->moving;
	arm_goal_reached = arm_state->goal_reached;
}

void ArmController::AeroStateMSG(const aero_srr_msgs::AeroStateConstPtr& aero_state)
{

	switch (aero_state->state)
	{

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
			previous_state = aero_state->state;

			break;
	}
}

int main(int argc, char **argv)
{

	/* Set up ROS */
	ros::init(argc, argv, "arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

//create the arm object
	ArmController arm(nh, param_nh);

	ros::spin();
}

