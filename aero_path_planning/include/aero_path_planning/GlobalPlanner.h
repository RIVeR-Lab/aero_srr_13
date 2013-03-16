/**
 * @file GlobalPlanner.h
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief \todo
 */

//License File

#ifndef AERO_GLOBAL_PLANNER_H_
#define AERO_GLOBAL_PLANNER_H_

//****************SYSTEM DEPENDANCIES**************************//
#include<tf/transform_listener.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>

//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/OryxPathPlanning.h>
#include<aero_path_planning/CarrotPathFinder.h>
//**********************NAMESPACES*****************************//

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

	void laserCB(const sensor_msgs::PointCloud2ConstPtr);

	/**
	 * @author Adam Panzica
	 * @brief  Converts lidar data to the global frame
	 * @param  [in]  scan_cloud   The cloud to convert
	 * @param  [out] result_cloud The resulting converted cloud
	 * @return True if frame can be converted, else false
	 */
	bool lidarToGlobal(const sensor_msgs::PointCloud2& scan_cloud, sensor_msgs::PointCloud2& result_cloud) const;

	/**
	 * @author Adam Panzica
	 * @brief  Converts a PointCloud2 message into an OccupancyGrid patch cloud of obstacles
	 * @param  [in]  scan_cloud   The point cloud message to convert
	 * @param  [out] result_cloud The aero_path_planning::PointCloud to store the resulting patch in
	 */
	void lidarMsgToOccGridPatch(const sensor_msgs::PointCloud2& scan_cloud, aero_path_planning::PointCloud& result_cloud) const;

	State       state_;

	std::string laser_topic_;
	std::string local_occupancy_topic_;
	std::string combined_odom_topic_;

	std::string global_frame_;
	std::string local_frame_;
	std::string lidar_frame_;

	int         local_x_size_;
	int         local_y_size_;
	int         global_x_size_;
	int         global_y_size_;

	aero_path_planning::CarrotPathFinder* path_planner_;

	ros::NodeHandle       nh_;
	tf::TransformListener transformer_;
	ros::NodeHandle       p_nh_;
	ros::Subscriber       joy_sub_;
	ros::Publisher        local_occ_pub_;
	ros::Subscriber       laser_sub_;
};

}; /* END aero_path_planning */


#endif /* AERO_GLOBAL_PLANNER_H_ */
