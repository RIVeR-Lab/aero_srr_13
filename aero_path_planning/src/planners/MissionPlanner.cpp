/**
 * @file   MissionPlanner.cpp
 *
 * @date   May 10, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planners/MissionPlanner.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;

#define LOAD_PARAM(nh, param_name, param_store, message_stream) if(!nh.getParam(param_name, param_store)) ROS_WARN_STREAM("Parameter "<<param_name<<" not set, using default value:"<<message_stream)

MissionPlanner::MissionPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh):
		nh_(nh),
		p_nh_(p_nh),
		transformer_(nh)
{
	ROS_INFO_STREAM("Misison Planner Starting Up...");
	this->loadParam();
	this->registerTopics();
	this->registerTimers();
	ROS_INFO_STREAM("Mission Planner Running!");
}

void MissionPlanner::loadParam()
{
	this->local_frame_        = "/base_footprint";
	this->global_frame_       = "/world";
	this->path_topic_         = "/path";
	this->state_topic_        = "/state";
	this->mission_goal_topic_ = "/mission_goal";
	this->path_goal_topic_    = "/path_goal";
	this->path_threshold_     = 1.0;
	LOAD_PARAM(this->p_nh_, "local_frame", this->local_frame_, this->local_frame_);
	LOAD_PARAM(this->p_nh_, "global_frame", this->global_frame_, this->global_frame_);
	LOAD_PARAM(this->p_nh_, "state_topic", this->state_topic_, this->state_topic_);
	LOAD_PARAM(this->p_nh_, "path_topic", this->path_topic_, this->path_topic_);
	LOAD_PARAM(this->p_nh_, "path_threshold", this->path_threshold_, this->path_threshold_<<"m");
	LOAD_PARAM(this->p_nh_, "mission_goal_topic", this->mission_goal_topic_, this->mission_goal_topic_);
	LOAD_PARAM(this->p_nh_, "path_goal_topic", this->path_goal_topic_, this->path_goal_topic_);

}

void MissionPlanner::registerTopics()
{
	this->state_sub_        = this->nh_.subscribe(this->state_topic_, 1,  &MissionPlanner::stateCB, this);
	this->path_sub_         = this->nh_.subscribe(this->path_topic_, 1, &MissionPlanner::pathCB, this);
	this->mission_goal_pub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->mission_goal_topic_, 1, true);
	this->path_goal_pub_    = this->nh_.advertise<geometry_msgs::PoseStamped>(this->path_goal_topic_, 1, true);
}

void MissionPlanner::registerTimers()
{

	this->goal_timer_ = this->nh_.createTimer(ros::Duration(1.0/10.0), &MissionPlanner::goalCB, this);
}
