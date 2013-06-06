/**
 * @file   Supervisor.cpp
 *
 * @date   Mar 22, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <aero_srr_msgs/AeroState.h>
#include <iostream>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_srr_supervisor/Supervisor.h>
#include "AeroSupervisorParameters.h"

//***********    NAMESPACES     ****************//

using namespace aero_srr_supervisor;

Supervisor::Supervisor(ros::NodeHandle& nh, ros::NodeHandle& p_nh):
		nh_(nh),
		p_nh_(p_nh)
{
	ROS_INFO_STREAM("Initializing Aero SRR Supervisor...");

	this->state_.state = state_t::STARTUP;
	this->loadParams();
	this->registerTopics();
	this->registerTimers();
	this->buildStateTable();
	this->stateUptd();
	ROS_INFO_STREAM("Aero SRR Supervisor Running!");
}

void Supervisor::loadParams()
{
	this->ctrlmd_topic_     = CTRLMD_TOPIC;
	this->aero_state_topic_ = STATE_TOPIC;

	if(!this->p_nh_.getParam(this->ctrlmd_topic_,   this->ctrlmd_topic_))     PARAM_WARN(this->ctrlmd_topic_, this->ctrlmd_topic_);

	if(!this->nh_.getParam(this->aero_state_topic_, this->aero_state_topic_)) PARAM_WARN(this->aero_state_topic_, this->aero_state_topic_);
}

void Supervisor::registerTopics()
{
	this->state_transition_srv_     = this->nh_.advertiseService(this->ctrlmd_topic_, &Supervisor::stateTransitionReqCB, this);
	this->aero_state_pub_ = this->nh_.advertise<aero_srr_msgs::AeroState>(this->aero_state_topic_, 2, true);
}

void Supervisor::registerTimers()
{

}

void Supervisor::buildStateTable()
{
	this->state_table_.addStateStringRepresentation(state_t::COLLECT, "COLLECT");
	this->state_table_.addStateStringRepresentation(state_t::ERROR, "ERROR");
	this->state_table_.addStateStringRepresentation(state_t::HOME, "HOME");
	this->state_table_.addStateStringRepresentation(state_t::MANUAL, "MANUAL");
	this->state_table_.addStateStringRepresentation(state_t::NAVOBJ, "NAVOBJ");
	this->state_table_.addStateStringRepresentation(state_t::PAUSE, "PAUSE");
	this->state_table_.addStateStringRepresentation(state_t::SAFESTOP, "SAFESTOP");
	this->state_table_.addStateStringRepresentation(state_t::SEARCH, "SEARCH");
	this->state_table_.addStateStringRepresentation(state_t::SHUTDOWN, "SHUTDOWN");
	this->state_table_.addStateStringRepresentation(state_t::STARTUP, "STARTUP");
	this->state_table_.addStateStringRepresentation(state_t::PICKUP, "PICKUP");

	this->state_table_.addTranstion(state_t::PAUSE, state_t::COLLECT, true);
	this->state_table_.addTranstion(state_t::PAUSE, state_t::ERROR, false);
	this->state_table_.addTranstion(state_t::PAUSE, state_t::HOME, true);
	this->state_table_.addTranstion(state_t::PAUSE, state_t::MANUAL, true);
	this->state_table_.addTranstion(state_t::PAUSE, state_t::NAVOBJ, true);
	this->state_table_.addTranstion(state_t::PAUSE, state_t::SAFESTOP, false);
	this->state_table_.addTranstion(state_t::PAUSE, state_t::SEARCH, true);
	this->state_table_.addTranstion(state_t::PAUSE, state_t::SHUTDOWN, false);
	this->state_table_.addTranstion(state_t::PAUSE, state_t::PICKUP, true);

	this->state_table_.addTranstion(state_t::STARTUP, state_t::MANUAL);
	this->state_table_.addTranstion(state_t::STARTUP, state_t::SEARCH);
	this->state_table_.addTranstion(state_t::STARTUP, state_t::SAFESTOP);
	this->state_table_.addTranstion(state_t::STARTUP, state_t::ERROR);


	this->state_table_.addTranstion(state_t::HOME, state_t::SAFESTOP);
	this->state_table_.addTranstion(state_t::HOME, state_t::ERROR);
	this->state_table_.addTranstion(state_t::HOME, state_t::MANUAL, true);

	this->state_table_.addTranstion(state_t::NAVOBJ, state_t::SEARCH, true);
	this->state_table_.addTranstion(state_t::NAVOBJ, state_t::MANUAL, true);
	this->state_table_.addTranstion(state_t::NAVOBJ, state_t::COLLECT, true);
	this->state_table_.addTranstion(state_t::NAVOBJ, state_t::SAFESTOP);

	this->state_table_.addTranstion(state_t::SEARCH, state_t::MANUAL, true);
	this->state_table_.addTranstion(state_t::SEARCH, state_t::ERROR);
	this->state_table_.addTranstion(state_t::SEARCH, state_t::SAFESTOP);

	this->state_table_.addTranstion(state_t::COLLECT, state_t::MANUAL, true);
	this->state_table_.addTranstion(state_t::COLLECT, state_t::SEARCH);
	this->state_table_.addTranstion(state_t::COLLECT, state_t::SAFESTOP, true);
	this->state_table_.addTranstion(state_t::COLLECT, state_t::ERROR);

	this->state_table_.addTranstion(state_t::SAFESTOP, state_t::SHUTDOWN);

	this->state_table_.addTranstion(state_t::ERROR, state_t::SAFESTOP, true);

	this->state_table_.addTranstion(state_t::PICKUP, state_t::COLLECT, true);
	this->state_table_.addTranstion(state_t::PICKUP, state_t::NAVOBJ);
	this->state_table_.addTranstion(state_t::PICKUP, state_t::SEARCH);
	this->state_table_.addTranstion(state_t::PICKUP, state_t::ERROR);
	this->state_table_.addTranstion(state_t::PICKUP, state_t::MANUAL);

}

bool Supervisor::stateTransitionReqCB(aero_srr_msgs::StateTransitionRequest::Request& request, aero_srr_msgs::StateTransitionRequest::Response& response)
{
	ROS_INFO_STREAM("Got a request to transition from current state:"<<this->state_<<"to new state:"<<request.requested_state);
	if(this->state_table_.isValidTransition(this->state_.state, request.requested_state.state))
	{
		ROS_INFO_STREAM("The request was granted!");
		this->state_     = request.requested_state;
		this->stateUptd();
		response.success = true;
	}
	else
	{
		ROS_INFO_STREAM("The reqest was denied!");
		std::string current_state;
		std::string requested_state;
		std::string transition_list;
		this->state_table_.stateToString(request.requested_state.state, requested_state);
		this->state_table_.stateToString(this->state_.state, current_state);
		this->buildTransitionList(this->state_, transition_list);
		response.error_message+="<";
		response.error_message+=current_state;
		response.error_message+="><";
		response.error_message+=requested_state;
		response.error_message+="><";
		response.error_message+=transition_list;
		response.error_message+=">";
		response.success = false;
	}
	return true;
}

void Supervisor::stateUptd() const
{
	ROS_INFO_STREAM("I'm transitioning to state:"<<this->state_);
	typedef aero_srr_msgs::AeroState state_t;
	aero_srr_msgs::AeroState message(this->state_);
	message.header.stamp = ros::Time::now();
	this->aero_state_pub_.publish(message);
}

void Supervisor::buildTransitionList(state_t state, std::string& list) const
{
	std::vector<uint8_t> transition_list;
	this->state_table_.getTransitionList(state.state, transition_list);
	BOOST_FOREACH(std::vector<uint8_t>::value_type state, transition_list)
	{
		std::string state_string;
		this->state_table_.stateToString(state, state_string);
		list+=" ";
		list+=state_string;
	}
}
