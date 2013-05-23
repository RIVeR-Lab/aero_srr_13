/**
 * @file   MissionPlanner.cpp
 *
 * @date   May 10, 2013
 * @author parallels
 * @brief  \TODO
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <boost/foreach.hpp>
#include <vector>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planners/MissionPlanner.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;

#define AERO_PATH_PLANNING_LOAD_PARAM(nh, param_name, param_store, message_stream) if(!nh.getParam(param_name, param_store)) ROS_WARN_STREAM("Parameter "<<param_name<<" not set, using default value:"<<message_stream)

MissionPlanner::MissionPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh):
				nh_(nh),
				p_nh_(p_nh),
				transformer_(nh),
				dr_server_(nh)
{
	ROS_INFO_STREAM("Misison Planner Starting Up...");
	this->loadParam();
	this->registerTopics();
	this->registerTimers();
	ROS_INFO_STREAM("Mission Planner Running!");
	//geometry_msgs::Pose mission_goal_one;
	//mission_goal_one.position.x = 10.0;
	//mission_goal_one.orientation.w = 1;
	//this->mission_goals_.push_back(mission_goal_one);
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
	AERO_PATH_PLANNING_LOAD_PARAM(this->p_nh_, "local_frame", this->local_frame_, this->local_frame_);
	AERO_PATH_PLANNING_LOAD_PARAM(this->p_nh_, "global_frame", this->global_frame_, this->global_frame_);
	AERO_PATH_PLANNING_LOAD_PARAM(this->p_nh_, "state_topic", this->state_topic_, this->state_topic_);
	AERO_PATH_PLANNING_LOAD_PARAM(this->p_nh_, "path_topic", this->path_topic_, this->path_topic_);
	AERO_PATH_PLANNING_LOAD_PARAM(this->p_nh_, "path_threshold", this->path_threshold_, this->path_threshold_<<"m");
	AERO_PATH_PLANNING_LOAD_PARAM(this->p_nh_, "mission_goal_topic", this->mission_goal_topic_, this->mission_goal_topic_);
	AERO_PATH_PLANNING_LOAD_PARAM(this->p_nh_, "path_goal_topic", this->path_goal_topic_, this->path_goal_topic_);

}

void MissionPlanner::registerTopics()
{
	this->state_sub_        = this->nh_.subscribe(this->state_topic_, 1,  &MissionPlanner::stateCB, this);
	this->path_sub_         = this->nh_.subscribe(this->path_topic_, 1, &MissionPlanner::pathCB, this);
	this->mission_goal_pub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->mission_goal_topic_, 1, true);
	this->path_goal_pub_    = this->nh_.advertise<geometry_msgs::PoseStamped>(this->path_goal_topic_, 1, true);
	this->dr_server_.setCallback(boost::bind(&MissionPlanner::drCB, this, _1, _2));
}

void MissionPlanner::registerTimers()
{

	this->goal_timer_ = this->nh_.createTimer(ros::Duration(1.0/10.0), &MissionPlanner::goalCB, this);
}

void MissionPlanner::drCB(const MissionPlannerConfig& config, uint32_t level)
{
	this->local_frame_   = config.local_frame;
	this->global_frame_  = config.global_frame;
	this->path_threshold_= config.path_threshold;
	if(!config.mission_goal_topic.compare(this->mission_goal_topic_))
	{
		this->mission_goal_topic_ = config.mission_goal_topic;
		this->mission_goal_pub_.shutdown();
		this->mission_goal_pub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->mission_goal_topic_, 1, true);
	}
	if(!config.path_goal_topic.compare(this->path_goal_topic_))
	{
		this->path_goal_topic_ = config.path_goal_topic;
		this->path_goal_pub_.shutdown();
		this->path_goal_pub_    = this->nh_.advertise<geometry_msgs::PoseStamped>(this->path_goal_topic_, 1, true);
	}
}

void MissionPlanner::pathCB(const nav_msgs::PathConstPtr& message)
{
	this->carrot_path_.clear();
	BOOST_FOREACH(std::vector<geometry_msgs::PoseStamped>::value_type pose, message->poses)
	{
		this->carrot_path_.push_back(pose);
	}
}

bool MissionPlanner::reachedNextGoal(const geometry_msgs::PoseStamped& worldLocation, const double threshold) const
{
	tf::Point world_point;
	tf::Point goal_point;
	tf::pointMsgToTF(worldLocation.pose.position, world_point);
	tf::pointMsgToTF(this->carrot_path_.front().pose.position, goal_point);
	double dist = world_point.distance(goal_point);
	ROS_INFO_STREAM_THROTTLE(10, "At position:"<<world_point<<"\nGoal position:"<<goal_point<<"\ndist="<<dist);
	return dist<threshold;
}

void MissionPlanner::goalCB(const ros::TimerEvent& event)
{
	geometry_msgs::PoseStamped current_point;

	if(this->calcRobotPointWorld(current_point))
	{
		if(!this->carrot_path_.empty())
		{
			if(this->reachedNextGoal(current_point, 1.25))
			{
				this->carrot_path_.pop_front();
				this->updateGoal();
			}
		}
		else
		{
			ROS_INFO_STREAM("Reached a Mission Goal, Moving to the next one!");
			if(!this->mission_goals_.empty())
			{
				this->mission_goals_.pop_front();
				this->updateMissionGoal();
			}
		}
	}
}

void MissionPlanner::updateGoal() const
{
	//ROS_INFO_STREAM("I'm Copying the Next Carrot Path Point Onto the Local Grid in frame "<<grid.getFrameId());
	if(!this->carrot_path_.empty())
	{
		geometry_msgs::PoseStamped goal_pose(this->carrot_path_.front());
		goal_pose.header.frame_id    = this->global_frame_;
		goal_pose.header.stamp       = ros::Time::now();
		goal_pose.pose.orientation.w = 1;
		this->path_goal_pub_.publish(goal_pose);
	}

}

bool MissionPlanner::calcRobotPointWorld(geometry_msgs::PoseStamped& point) const
{
	bool success = true;
	geometry_msgs::PoseStamped robot_pose;
	robot_pose.header.frame_id    = this->local_frame_;
	robot_pose.header.stamp       = ros::Time::now();
	robot_pose.pose.orientation.w = 1;

	//look up the robot's position in the world frame
	try
	{
		this->transformer_.waitForTransform(this->global_frame_, robot_pose.header.frame_id, robot_pose.header.stamp, ros::Duration(0.1));
		this->transformer_.transformPose(this->global_frame_, robot_pose, point);
	}
	catch(std::exception& e)
	{
		ROS_ERROR_STREAM_THROTTLE(1, e.what());
		success = false;
	}
	return success;
}

void MissionPlanner::stateCB(const aero_srr_msgs::AeroStateConstPtr& message)
{
	typedef aero_srr_msgs::AeroState state_t;
	switch(message->state)
	{
	//	case state_t::ERROR:
	//	case state_t::MANUAL:
	//	case state_t::PAUSE:
	//	case state_t::SAFESTOP:
	//	case state_t::SHUTDOWN:
	//	case state_t::COLLECT:
	//	case state_t::SEARCH:
	//	case state_t::NAVOBJ:
	//	default:
	//		ROS_ERROR_STREAM("Received Unkown State: "<<*message);
	//		break;
	}
}


void MissionPlanner::updateMissionGoal() const
{
	if(!this->mission_goals_.empty())
	{
		geometry_msgs::PoseStampedPtr message(new geometry_msgs::PoseStamped());
		message->pose            = this->mission_goals_.front();
		message->header.frame_id = this->global_frame_;
		message->header.stamp    = ros::Time::now();
	}
}
