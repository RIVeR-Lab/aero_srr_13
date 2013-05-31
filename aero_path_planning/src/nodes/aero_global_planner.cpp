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
#include<deque>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/planning_strategies/RRTCarrot.h>
#include<aero_path_planning/planners/GlobalPlanner.h>
#include<aero_path_planning/occupancy_grid/OccupancyGrid.h>
#include<aero_path_planning/planning_strategies/AStarCarrot.h>
//**********************NAMESPACES*****************************//

using namespace aero_path_planning;

bool collisionCheck(const aero_path_planning::Point& point, const aero_path_planning::OccupancyGrid& map)
{
	try
	{
		if(map.getPointTrait(point)==aero_path_planning::OBSTACLE)
		{
			return true;
		}
	}
	catch(std::exception& e)
	{
		return true;
	}
	return false;
}

int main(int argc, char **argv) {
//	ros::init(argc, argv, "areo_global_planner");
//	ros::NodeHandle nh;
//	ros::NodeHandle p_nh("~");
//	AStarCarrot path_planner;
//	//path_planner.seedSampler(ros::Time::now().toNSec());
//	CarrotPathFinder::collision_func_ cf = boost::bind(&collisionCheck, _1, _2);
//	path_planner.setCollision(cf);
//
//	//Build a test occupancy grid
//	Point origin;
//	origin.x = 0;
//	origin.y = 50;
//	origin.z = 0;
//	OccupancyGrid testGrid(100, 100, .25, origin, aero_path_planning::FREE_LOW_COST);
//	PointCloud obstacle;
//	Point start;
//	start.x = 50;
//	start.y = -30;
//	start.z = 0;
//	Point end;
//	end.x   = 50;
//	end.y   = 30;
//	end.z   = 0;
//	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
//	start.x = 51;
//	end.x   = 51;
//	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
//	start.x = 52;
//	end.x   = 52;
//	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
//	start.x = 52;
//	end.x   = 52;
//	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
//	start.x = 53;
//	end.x   = 53;
//	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
//	start.x = 54;
//	end.x   = 54;
//	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
//	start.x = 70;
//	end.x   = 30;
//	end.y   = 30;
//	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
//	start.x = 71;
//	end.x   = 31;
//	end.y   = 30;
//	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
//
//	BOOST_FOREACH(Point point, obstacle)
//	{
//		testGrid.setPointTrait(point);
//	}
//
//	path_planner.setCarrotDelta(1);
//	path_planner.setSearchMap(testGrid);
//
//	Point start_point;
//	start_point.x = 0;
//	start_point.y = 0;
//	start_point.z = 0;
//	start_point.rgba = aero_path_planning::ROBOT;
//	Point goal_point;
//	goal_point.x  = 90;
//	goal_point.y  = 0;
//	goal_point.z  = 0;
//	goal_point.rgba = aero_path_planning::GOAL;
//	std::deque<Point> path;
//	ros::Duration timeout(1);
//
//	while(ros::ok())
//	{
//		OccupancyGrid copyGrid(testGrid);
//		if(path_planner.search(start_point, goal_point, timeout , path)) ROS_INFO_STREAM("I Found A Solution!");
//		else ROS_INFO("I Timed Out Or Solution Couldn't Be Found");
//		PointCloud path_cloud;
//		while(path.size()!=0)
//		{
//			path_cloud.push_back(path.front());
//			path.pop_front();
//		}
//
//		BOOST_FOREACH(Point point, path_cloud)
//		{
//			//ROS_INFO_STREAM("I'm Printing Point ("<<point.x<<","<<point.y<<")");
//			point.rgba = aero_path_planning::TENTACLE;
//			copyGrid.setPointTrait(point);
//		}
//		copyGrid.setPointTrait(goal_point);
//		copyGrid.setPointTrait(start_point);
//
//		ROS_INFO_STREAM("\n"<<*(copyGrid.toString(0,0)));
//		std::string input;
//		std::cin >> input;
//	}
//
//	GlobalPlanner planner(nh, p_nh, path_planner);
}



