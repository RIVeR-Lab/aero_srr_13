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
#include<aero_path_planning/OccupancyGrid.h>
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
	ros::init(argc, argv, "areo_global_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");
	RRTCarrot path_planner(5);
	CarrotPathFinder::collision_func_ cf = boost::bind(&collisionCheck, _1, _2);
	path_planner.setCollision(cf);

	//Build a test occupancy grid
	Point origin;
	origin.x = 0;
	origin.y = 50;
	origin.z = 0;
	OccupancyGrid testGrid(200, 200, .25, origin, aero_path_planning::FREE_LOW_COST);
	PointCloud obstacle;
	Point start;
	start.x = 50;
	start.y = -20;
	start.z = 0;
	Point end;
	end.x   = 50;
	end.y   = 20;
	end.z   = 0;
	castLine(start, end, aero_path_planning::OBSTACLE, obstacle);
	BOOST_FOREACH(Point point, obstacle)
	{
		testGrid.setPointTrait(point, aero_path_planning::OBSTACLE);
	}

	path_planner.setCarrotDelta(5);
	path_planner.setSearchMap(testGrid);

	Point start_point;
	start_point.x = 0;
	start_point.y = 0;
	start_point.z = 0;
	Point goal_point;
	goal_point.x  = 90;
	goal_point.y  = 0;
	goal_point.z  = 0;
	std::queue<Point> path;
	ros::Duration timeout(1000);
	if(path_planner.search(start_point, goal_point, timeout , path)) ROS_INFO_STREAM("I Found A Solution!");
	else ROS_INFO("I Timed Out Or Solution Couldn't Be Found");
	PointCloud path_cloud;
	while(path.size()!=0)
	{
		path_cloud.push_back(path.back());
		path.pop();
	}

	ros::Publisher path_pub =  nh.advertise<sensor_msgs::PointCloud2>("test_path", 2);
	while(ros::ok())
	{
		sensor_msgs::PointCloud2 message;
		pcl::toROSMsg(path_cloud,message);
		path_pub.publish(message);
		ros::spinOnce();
	}

	GlobalPlanner planner(nh, p_nh, path_planner);
}



