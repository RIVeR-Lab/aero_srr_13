/**
 * @file   aero_mission_planner.cpp
 *
 * @date   May 10, 2013
 * @author Adam Panzica
 * @brief  Implementation of the aero_mission_planner node
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planners/MissionPlanner.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "aero_mission_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");
	MissionPlanner mp(nh, p_nh);
	ros::spin();
}
