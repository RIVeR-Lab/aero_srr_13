/**
 * @file   Supervisor.cpp
 *
 * @date   Mar 22, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <aero_srr_msgs/AeroState.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_srr_supervisor/Supervisor.h>
#include "AeroSupervisorParameters.h"

//***********    NAMESPACES     ****************//

using namespace aero_srr_supervisor;

Supervisor::Supervisor(ros::NodeHandle& nh, ros::NodeHandle& p_nh):
		state_(STARTUP),
		nh_(nh),
		p_nh_(p_nh)
{
	ROS_INFO_STREAM("Initializing Aero SRR Supervisor...");
	this->loadParams();
	this->registerTopics();
	this->registerTimers();
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
	this->ctrlmd_sub_     = this->nh_.subscribe(this->ctrlmd_topic_, 2, &Supervisor::setCtrlMdCB, this);
	this->aero_state_pub_ = this->nh_.advertise<aero_srr_msgs::AeroState>(this->aero_state_topic_, 2, true);
}

void Supervisor::setCtrlMdCB(const aero_srr_supervisor::SetControlModeConstPtr& message)
{
	typedef aero_srr_supervisor::SetControlMode mode_t;
	switch(message->mode)
	{
	case mode_t::MANUAL:
		this->state_ = MANUAL;
		break;
	case mode_t::AUTONIMOUS:
		this->state_ = SEARCHING;
		break;
		break;
	}
	this->stateUptd();
}

void Supervisor::stateUptd() const
{
	typedef aero_srr_msgs::AeroState state_t;
	aero_srr_msgs::AeroState message;
	switch(this->state_)
	{
	case ERROR:
		message.state = state_t::ERROR;
		break;
	case MANUAL:
		message.state = state_t::MANUAL;
		break;
	case SEARCHING:
		message.state = state_t::SEARCH;
		break;
	case NAVOBJ:
		message.state = state_t::NAVOBJ;
		break;
	case HOME:
		message.state = state_t::HOME;
		break;
	case PAUSE:
		message.state = state_t::PAUSE;
		break;
	case SAFESTOP:
		message.state = state_t::SAFESTOP;
		break;
	case SHUTDOWN:
		message.state = state_t::SHUTDOWN;
		break;
	case STARTUP:
		message.state = state_t::STARTUP;
		break;
	case COLLECT:
		message.state = state_t::COLLECT;
		break;
	default:
		message.state = -1;
		break;
	}
	message.header.stamp = ros::Time::now();

	this->aero_state_pub_.publish(message);
}
