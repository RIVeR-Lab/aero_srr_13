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
	/**
	 * @author Adam Panzica
	 * @brief  Enum of states the GlobalPlanner can be in
	 */
	typedef enum GPState_t
	{
		MANUAL,                //!< MANUAL                  Global Planner is in manual override
		AUTONIMOUS_SEARCH,     //!< AUTONIMOUS_SEARCH       Global Planner is searching the map for objects
		AUTONIMOUS_GO_ITEM,    //!< AUTONIMOUS_GO_ITEM      Global Planner is navigating to an object of interest
		AUTONIMOUS_PICKUP_ITEM,//!< AUTONIMOUS_PICKUP_ITEM  Global Planner is picking up an object of interest
		AUTONIMOUS_HOME        //!< AUTONIMOUS_HOME         Global Planner is navigating to the start platform
	} State;

public:
	GlobalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh, aero_path_planning::CarrotPathFinder& path_planner_);
	virtual ~GlobalPlanner();

private:

	/**
	 * @author Adam Panzica
	 * @brief  Loads parameters from the ROS system
	 */
	void loadOccupancyParam();

	/**
	 * @author Adam Panzica
	 * @brief  Registers topics with the ROS system
	 */
	void registerTopics();

	/**
	 * @author Adam Panzica
	 * @brief  Register timers with the ROS system
	 */
	void registerTimers();

	/**
	 * @author Adam Panzica
	 * @brief  Builds the initial global map
	 */
	void buildGlobalMap();

	/**
	 * @author Adam Panzica
	 * @brief  Callback for re-planning at the global scale
	 * @param event
	 */
	void planCB(const ros::TimerEvent& event);

	/**
	 * @author Adam Panzica
	 * @brief  Sets the global planner to manual mode
	 */
	void setManual();

	/**
	 * @author Adam Panzica
	 * @brief  Callback for processing LIDAR scans
	 * @param message
	 */
	void laserCB(const sensor_msgs::PointCloud2ConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief  Callback for handling Odometry messages
	 * @param message
	 *
	 * This callback handles advancing the CarrotPath
	 */
	void odomCB(const nav_msgs::OdometryConstPtr& message);

	/**
	 * @brief Adam Panzica
	 * @brief Chunks the global map into a local OccupancyGrid and sends it to the local planner
	 * @param event
	 */
	void chunckCB(const ros::TimerEvent& event);

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

	std::string laser_topic_;           ///Topic name for receiving LIDAR point clouds
	std::string local_occupancy_topic_; ///Topic name for communicating with the local planner
	std::string odom_topic_;            ///Topic name for receiving odometry data
	std::string state_topic_;           ///Topic name for receiving robot state from the supervisor

	std::string global_frame_;			///frame_id of the global scale frame
	std::string local_frame_;           ///frame_id of the local scale frame
	std::string lidar_frame_;           ///frame_id of the LIDAR frame

	int         local_x_size_;			///Size of the x axis of the local OccupancyGrid, in grid units
	int         local_y_size_;          ///Size of the y axis of the local OccupancyGrid, in grid units
	int         local_z_size_;          ///Size of the z axis of the local OccupancyGrid, in grid units
	int         local_x_ori_;           ///Location of the x origin of the local OccupancyGrid, in grid units
	int         local_y_ori_;           ///Location of the y origin of the local OccupancyGrid, in grid units
	int         local_z_ori_;           ///Location of the z origin of the local OccupancyGrid, in grid units
	double      local_res_;             ///Local occupancy grid resolution
	double      local_update_rate_;     ///Update frequency for generating new local grids
	int         global_x_size_;			///Size of the x axis of the global OccupancyGrid, in grid units
	int         global_y_size_;			///Size of the y axis of the global OccupancyGrid, in grid units
	int         global_z_size_;			///Size of the z axis of the global OccupancyGrid, in grid units
	int         global_x_ori_;          ///Location of the x origin of the global OccupancyGrid, in grid units
	int         global_y_ori_;          ///Location of the y origin of the global OccupancyGrid, in grid units
	int         global_z_ori_;          ///Location of the z origin of the global OccupancyGrid, in grid units
	double      global_res_;            ///Global occupancy grid resolution
	double      global_update_rate_;    ///Update frequency for planning on the global path

	nav_msgs::Odometry    last_odom_;   ///The odometry data received by the planner

	aero_path_planning::CarrotPathFinder* path_planner_;  ///The current global planner strategy
	aero_path_planning::OccupancyGridPtr  global_map_;    ///The global OccupancyGrid
	std::queue<Point>                     carrot_path_;   ///The current set of points on the global path
	double                                path_threshold_;///The threshold for determining we've gotten to a point on the path, in grid units_

	ros::NodeHandle       nh_;            ///Global NodeHandle into the ROS system
	ros::NodeHandle       p_nh_;          ///Private NodeHandle into the ROS system
	tf::TransformListener transformer_;   ///Hook into the tf system
	ros::Subscriber       joy_sub_;       ///Subscriber for joy messages
	ros::Publisher        local_occ_pub_; ///Publisher to send OccupancyGrids to the local planner
	ros::Subscriber       laser_sub_;     ///Subscriber for LIDAR scans
	ros::Subscriber       odom_sub_;      ///Subscriber for Odometry messages
	ros::Subscriber       state_sub;      ///Subscriber for the supervisor state
	ros::Timer            chunck_timer_;  ///Timer to chunk global map into local map
	ros::Timer            plan_timer_;    ///Timer to plan on the global map
	ros::Duration         plan_timerout_; ///Timeout to produce global plans
};

}; /* END aero_path_planning */


#endif /* AERO_GLOBAL_PLANNER_H_ */
