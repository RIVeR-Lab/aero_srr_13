/**
 * @file global_planner.cpp
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//

//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/GlobalPlanner.h>
#include<aero_path_planning/RRTCarrot.h>
//**********************NAMESPACES*****************************//


using namespace aero_path_planning;

GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh, aero_path_planning::CarrotPathFinder& path_planner):
		state_(MANUAL),
		path_planner_(path_planner),
		nh_(nh),
		p_nh_(p_nh)
{
	ROS_INFO("Initializing Global Planner...");

	ROS_INFO("Loading Occupancy Grid Parameters...");

	ROS_INFO("Registering Topics...");


	ROS_INFO("Aero Global Planner Running!");

}

GlobalPlanner::~GlobalPlanner()
{

}

void GlobalPlanner::loadOccupancyParam()
{

}

void GlobalPlanner::registerTopics()
{

}
