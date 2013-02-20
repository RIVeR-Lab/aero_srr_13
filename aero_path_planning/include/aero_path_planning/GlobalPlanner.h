/**
 * @file global_planner.h
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<tf/tf.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>

//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/OryxPathPlanning.h>
#include<aero_path_planning/CarrotPathFinder.h>
//**********************NAMESPACES*****************************//

#ifndef AERO_GLOBAL_PLANNER_H_
#define AERO_GLOBAL_PLANNER_H_

namespace aero_path_planning
{

class GlobalPlanner
{
public:
	typedef enum GPState_t
	{
		MANUAL,
		AUTONIMOUS_SEARCH,
		AUTONIMOUS_GO_ITEM,
		AUTONIMOUS_PICKUP_ITEM,
		AUTONIMOUS_HOME
	} State;

public:
	GlobalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh, aero_path_planning::CarrotPathFinder& path_planner_);
	virtual ~GlobalPlanner();

private:
	void loadOccupancyParam();
	void registerTopics();
	void buildGlobalMap();

	void planCB(const ros::TimerEvent& event);
	void setManual();

	State       state_;

	std::string laser_topic_;
	std::string local_occupancy_topic_;
	std::string combined_odom_topic_;

	int         local_x_size_;
	int         local_y_size_;
	int         global_x_size_;
	int         global_y_size_;

	aero_path_planning::CarrotPathFinder* path_planner_;

	tf::Transformer     transformer_;
	ros::NodeHandle     nh_;
	ros::NodeHandle     p_nh_;
	ros::Subscriber     joy_sub_;

};

}; /* END aero_path_planning */


#endif /* AERO_GLOBAL_PLANNER_H_ */
