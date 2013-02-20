/**
 * @file aero_global_planner.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<boost/bind.hpp>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/RRTCarrot.h>
#include<aero_path_planning/GlobalPlanner.h>
//**********************NAMESPACES*****************************//

using namespace aero_path_planning;

bool collisionCheck(const aero_path_planning::Point& point, const aero_path_planning::OccupancyGrid& map)
{
	if(map.getPointTrait(point)==aero_path_planning::OBSTACLE)
	{
		return true;
	}
	return false;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "areo_global_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");
	RRTCarrot path_planner(1);
	CarrotPathFinder::collision_func_ cf = boost::bind(&collisionCheck, _1, _2);
	path_planner.setCollision(cf);
	GlobalPlanner planner(nh, p_nh, path_planner);
}



