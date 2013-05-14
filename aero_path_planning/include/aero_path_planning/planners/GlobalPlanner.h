/**
 * @file GlobalPlanner.h
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief Class definition for GlobalPlanner
 */

//License File

#ifndef AERO_GLOBAL_PLANNER_H_
#define AERO_GLOBAL_PLANNER_H_

//****************SYSTEM DEPENDANCIES**************************//
#include<tf/transform_listener.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include <aero_srr_msgs/AeroState.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/utilities/AeroPathPlanning.h>
#include<aero_path_planning/planning_strategies/CarrotPathFinder.h>
//**********************NAMESPACES*****************************//

namespace sm  = sensor_msgs;
namespace nm  = nav_msgs;
namespace app = aero_path_planning;

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
	GlobalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh, app::CarrotPathFinder& path_planner_);
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
	 * @brief  Sets/unsets to manual mode
	 * @param [in] enable true for manual, false for autonomous
	 */
	void setManual(bool enable);

	/**
	 * @author Adam Panzica
	 * @brief Sets the GlobalPlanner to search mode
	 */
	void setSearch();

	/**
	 * @author Adam Panzica
	 * @brief Sets the GlobalPLanner to navigate to an object of interest
	 */
	void setNavObj();

	/**
	 * @author Adam Panzica
	 * @brief  Callback for processing LIDAR scans
	 * @param message
	 */
	void laserCB(const sm::PointCloud2ConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief Callback for handling updates from SLAM
	 * @param message
	 */
	void slamCB(const nm::OccupancyGridConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief Chunks the global map into a local OccupancyGrid and sends it to the local planner
	 * @param event
	 */
	void chunckCB(const ros::TimerEvent& event);

	/**
	 * @author Adam Panzica
	 * @brief Callback for handling changes in robot state
	 * @param message
	 */
	void stateCB(const aero_srr_msgs::AeroStateConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief Callback for handling getting a new mission goal
	 * @param message
	 */
	void missionGoalCB(const geometry_msgs::PoseStampedConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief Sends the global map out for vizualization
	 */
	void visualizeMap() const;



	/**
	 * @author Adam Panzica
	 * @brief  Converts lidar data to the global frame
	 * @param  [in]  scan_cloud   The cloud to convert
	 * @param  [out] result_cloud The resulting converted cloud
	 * @return True if frame can be converted, else false
	 */
	bool lidarToGlobal(const sm::PointCloud2& scan_cloud, sm::PointCloud2& result_cloud) const;

	/**
	 * @author Adam Panzica
	 * @brief  Converts a PointCloud2 message into an OccupancyGrid patch cloud of obstacles
	 * @param  [in]  scan_cloud   The point cloud message to convert
	 * @param  [out] result_cloud The aero_path_planning::PointCloud to store the resulting patch in
	 */
	void lidarMsgToOccGridPatch(const sm::PointCloud2& scan_cloud, app::PointCloud& result_cloud) const;

	/**
	 * @author Adam Panzica
	 * @brief  The collision function to use for determining valid locations
	 * @param [in] point The point to check for collision
	 * @param [in] map   The map to check against for collision
	 * @return True if in collision, else false
	 */
	bool checkCollision(const app::Point& point, const app::OccupancyGrid& map) const;

	/**
	 * @author Adam Panzica
	 * @brief Converts the carrot path to a nav_msgs:Path message
	 * @param path The message to fill
	 */
	void carrotToPath(nav_msgs::Path& path) const;

	/**
	 * @author Adam Panzica
	 * @brief looks up the robot's current location in the world frame
	 * @param [in] point Point to fill with the robot's location (in meters, not grid coordinates)
	 * @return true if the transform was sucessful, else false
	 */
	bool calcRobotPointWorld(app::Point& point) const;

	aero_srr_msgs::AeroState       state_;
	app::Point  current_point_;

	std::string global_laser_topic_;    ///Topic name for receiving LIDAR point clouds
	std::string local_occupancy_topic_; ///Topic name for communicating with the local planner
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

	CarrotPathFinder::collision_func_ cf_;            ///The collision function
	CarrotPathFinder*                 path_planner_;  ///The current global planner strategy
	OccupancyGridPtr                  global_map_;    ///The global OccupancyGrid
	std::deque<Point>                 carrot_path_;   ///The current set of points on the global path
	geometry_msgs::PoseStamped        mission_goal_; ///The current mission-goal to plan to

	ros::NodeHandle       nh_;            ///Global NodeHandle into the ROS system
	ros::NodeHandle       p_nh_;          ///Private NodeHandle into the ROS system
	tf::TransformListener transformer_;   ///Hook into the tf system
	ros::Publisher        local_occ_pub_; ///Publisher to send OccupancyGrids to the local planner
	ros::Publisher        map_viz_pub_;   ///Publisher to visualizing the global map
	ros::Publisher        path_pub_;      ///Publisher to visualize the global path
	ros::Subscriber       laser_sub_;     ///Subscriber for LIDAR scans
	ros::Subscriber       state_sub;      ///Subscriber for the supervisor state
	ros::Subscriber       slam_sub_;      ///Subscriber to nav_msgs::OccupancyGrid messages from SLAM
	ros::ServiceClient    state_client_;  ///Client to request state transitions
	ros::Subscriber       mission_goal_sub_; ///Subscriber to new mission goals

	ros::Timer            chunck_timer_;  ///Timer to chunk global map into local map
	ros::Timer            plan_timer_;    ///Timer to plan on the global map

	ros::Duration         plan_timerout_; ///Timeout to produce global plans
};

}; /* END aero_path_planning */


#endif /* AERO_GLOBAL_PLANNER_H_ */
