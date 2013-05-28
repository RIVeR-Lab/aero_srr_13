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
#include <aero_control/boom_controller.h>

using namespace aero_control;

BoomController::BoomController(ros::NodeHandle nh, ros::NodeHandle param_nh)
{
	std::string boom_control("boom_control"); ///String containing the topic name for object position
	std::string aero_state("aero_state"); ///String containing the topic name for AeroState
	std::string aero_state_transition("aero_state_transition"); ///String containing the topic name for AeroStateTransition

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(boom_control, boom_control))
		ROS_WARN("Parameter <%s> Not Set. Using Default Boom Control Topic <%s>!", boom_control.c_str(),
				boom_control.c_str());
	if (!param_nh.getParam(aero_state, aero_state))
		ROS_WARN("Parameter <%s> Not Set. Using Default Aero State Topic <%s>!", aero_state.c_str(),
				aero_state.c_str());

	if (!param_nh.getParam(aero_state_transition, aero_state_transition))
		ROS_WARN("Parameter <%s> Not Set. Using Default Aero State Transition Topic <%s>!",
				aero_state_transition.c_str(), aero_state_transition.c_str());

	//Print out received topics
	ROS_DEBUG("Using Boom Control Topic Name: <%s>", boom_control.c_str());
	ROS_DEBUG("Using Aero State Topic Name: <%s>", aero_state.c_str());
	ROS_DEBUG("Using Aero State Transition Topic Name: <%s>", aero_state_transition.c_str());

	ROS_INFO("Starting Up Boom Controller...");

	this->active_state = false;
	this->pause_state = false;
	this->boom_path_step_num = 0;

	/* Messages */
	this->aero_state_sub = nh.subscribe(aero_state, 1, &BoomController::AeroStateMSG, this);

	/* Services */
	this->aero_state_transition_srv_client = nh.serviceClient<aero_srr_msgs::StateTransitionRequest>(
			aero_state_transition);
	this->boom_control_srv_client = nh.serviceClient<aero_base::SetBoomPosition>(boom_control);
	this->PlanBoomPath();
	
	ros::Rate home_rate(1);
	
	while (ros::ok())
	{
		ROS_INFO("Entered While Loop!");

		ROS_INFO("Active State = %d, Pause State = %d",this->active_state,this->pause_state);


		if (this->active_state == true && this->pause_state != true)
		{
			ROS_INFO("Active!");

			GoToPosition(this->boom_path[boom_path_steps].angle,this->boom_path[boom_path_steps].velocity ,this->boom_path[boom_path_steps].delay);
			boom_path_step_num++;
			if( boom_path_step_num >=boom_path_steps)
			{
				boom_path_step_num = 0;
			}

		} else if (this->pause_state != true)
		{
			ROS_INFO("Home!");

			GoHome();
			home_rate.sleep();
		}
		ROS_INFO("Spinning!");

		ros::spinOnce();
		ros::Duration(2.0).sleep();

	}

	GoHome();
	ROS_INFO("Done!");

}

void BoomController::SendBoomControl(aero_base::SetBoomPosition boom_position)
{
	boom_control_srv_client.call(boom_position);
}

void BoomController::GoToPosition(double angle, double velocity, double delay)
{

	aero_base::SetBoomPosition boom_position;
	boom_position.request.angle = angle;
	boom_position.request.max_velocity = velocity;
	SendBoomControl(boom_position);
	ros::Duration(delay).sleep();

}

void BoomController::GoHome(void)
{
	GoToPosition(0,1,0);
}

void BoomController::AeroStateMSG(const aero_srr_msgs::AeroStateConstPtr& aero_state)
{
	ROS_INFO("State = %d",aero_state->state);

	switch (aero_state->state)
	{


		case aero_srr_msgs::AeroState::SEARCH:
			this->active_state = true;
			this->pause_state = false;
			break;
		case aero_srr_msgs::AeroState::SHUTDOWN:
			this->active_state = false;
			this->pause_state = false;
			ros::shutdown();
			break;
		case aero_srr_msgs::AeroState::PAUSE:
			this->pause_state = true;
			break;
		default:
			this->active_state = false;
			this->pause_state = false;
			break;
	}
}

int main(int argc, char **argv)
{

	/* Set up ROS */
	ros::init(argc, argv, "boom_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

//create the arm object
	BoomController arm(nh, param_nh);

}

