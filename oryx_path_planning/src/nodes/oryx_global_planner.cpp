/**
 * @file	oryx_global_planner.cpp
 * @date	Jan 1, 2013
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

//*********************SYSTEM DEPENDENCIES**********************//
#include<ros/ros.h>

//*********************LOCAL DEPENDENCIES**********************//
#include"OryxPathPlanning.h"
#include"OryxPathPlannerConfig.h"


using namespace oryx_path_planning;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "oryx_global_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	std::string local_planner_topic("occupancy_point_cloud_topic");
	std::string odometry_topic("odometry_topic");

	//Node Information Printout
		ROS_INFO("Starting Up Oryx Global Planner Version %d.%d.%d", oryx_path_planner_VERSION_MAJOR, oryx_path_planner_VERSION_MINOR, oryx_path_planner_VERSION_BUILD);

	//Get Private Parameters
	if(!p_nh.getParam(local_planner_topic,local_planner_topic))	PARAM_WARN(local_planner_topic,	local_planner_topic);
	if(!p_nh.getParam(odometry_topic,	odometry_topic))		PARAM_WARN(odometry_topic,		odometry_topic);
}
