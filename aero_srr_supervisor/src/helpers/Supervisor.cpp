/**
 * @file   Supervisor.cpp
 *
 * @date   Mar 22, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
//************ LOCAL DEPENDANCIES ****************//
#include <aero_srr_supervisor/Supervisor.h>

//***********    NAMESPACES     ****************//

using namespace aero_srr_supervisor;

Supervisor::Supervisor(ros::NodeHandle& nh, ros::NodeHandle& p_nh):
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
	this->ctrlmd_topic_ = "aero/supervisor/control_mode_topic";

	if(!this->p_nh_.getParam(this->ctrlmd_topic_, this->ctrlmd_topic_)) PARAM_WARN(this->ctrlmd_topic_, this->ctrlmd_topic_);
}

void Supervisor::registerTopics()
{
	this->ctrlmd_sub_ = this->nh_.subscribe(this->ctrlmd_topic_, 2, &Supervisor::setCtrlMdCB, this);
}

void Supervisor::registerTimers()
{

}

void Supervisor::setCtrlMdCB(const aero_srr_supervisor::SetControlModeConstPtr& message)
{

}
