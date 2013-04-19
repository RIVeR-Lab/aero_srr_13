/**
 * @file LocalPlanner.h
 *
 * @date Mar 9, 2013
 * @author aruis
 */

#ifndef LOCALPLANNER_H_
#define LOCALPLANNER_H_

//********************** SYSTEM DEPENDANCIES **********************//
#include<ros/ros.h>
#include<queue>
#include<actionlib/client/simple_action_client.h>
#include<boost/circular_buffer.hpp>
#include<aero_srr_msgs/SoftwareStop.h>
#include<geometry_msgs/Twist.h>
//********************** LOCAL  DEPENDANCIES **********************//
#include <aero_path_planning/utilities/AeroPathPlanning.h>
#include <aero_path_planning/OccupancyGridMsg.h>
#include <aero_srr_msgs/AeroState.h>

namespace aero_path_planning
{
class LocalPlanner
{
public:
	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new local planner
	 * @param [in] nh   Public node handle into ROS system
	 * @param [in] p_nh Private node handle into ROS system
	 * @throw std::runtime_error	If the planner is unable to connect to the base platform
	 */
	LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh) throw(std::runtime_error);
	/**
	 * default destructor
	 */
	virtual ~LocalPlanner();

private:

	int		platform_;		    ///flat marking what platform we're running on
	bool	should_plan_;	    ///Flag for signaling if the local planner should be running
	bool    tentacle_mode_;     ///Flag for signalling if the local planner should be running in tetacle mode

	double	goal_weight_;	    ///weighting factor to bias tentacle selection towards the goal point
	double	trav_weight_;	    ///weighting factor to bias tentacle selection away from previously traversed points
	double	diff_weight_;	    ///weighting factor to bias tentacle selection away from difficult terrain
	double  unkn_weight_;	    ///weighting factor to bias tentacle selection towards unknown terrain
	double	x_dim_;		        ///x dimension of the occupancy grid to use, in real units
	double	y_dim_;		        ///y dimension of the occupancy grid to use, in real units
	double	z_dim_;		        ///z dimension of the occupancy grid to use, in real units
	double	res_;		        ///resolution the occupancy grid to use, in real units per grid unit
	double	current_vel_;	    ///Current Velocity of the Platform
	double	current_rad_;	    ///Current Radius followed by the Platform
	double  set_vel_;	        ///The target velocity to set the robot to
	double  set_rad_;	        ///The target radius to set the robot to

	std::string	v_action_topic_;///Actionlib topic name to send velocity commands over
	std::string pc_topic_;		///topic name of the ROS topic to receive new occupancy grid data over
	std::string state_topic_;   ///Topic name of the ROS topic to receive new AeroState messages over
	std::string man_topic_;     ///Topic name of the ROS topic to receive Joy messages over
	std::string lidar_topic_;   ///Topic name of the ROS topic to receive LIDAR data over

	ros::NodeHandle nh_;	    ///Node handle for publishing/subscribing to topics
	ros::NodeHandle p_nh_;      ///Nodes handle to load private params
	ros::Subscriber pc_sub_;    ///Subscriber to the ROS topic to receive new occupancy grid data over
	ros::Subscriber state_sub_; ///Subscriber to the ROS topic to receive AeroState messages
	ros::Subscriber joy_sub_;   ///Subscriber to the ROS topic to receive Joy messages
	ros::Subscriber	stop_sub_;	///Subscriber to the ROS topic to receive the software stop message
	ros::Subscriber lidar_sub_; ///Subscriber to the ROS topic to receive local LIDAR data over
	ros::Publisher	vel_pub_;	///Publisher for Twist messages to a platform that takes them
	ros::Publisher  tent_pub_;  ///Publisher for visualizing selected tentacles
	ros::Timer      vel_timer_;	///Timer that will send velocity updates to the platform at a constant rate
	ros::Timer      plan_timer_;///Timer that will attempt to select a new tentacle at a constant rate

	aero_path_planning::Point	origin_;	///The origin to use for the occupancy grids
	TentacleGeneratorPtr tentacles_;	///Pointer to the tentacle generator which contains the tentacles to use for planning


	boost::circular_buffer<OccupancyGrid > occupancy_buffer_;	///Buffer to store received OccupancyGrid data

	/**
	 * @author	Adam Panzica
	 * @brief	Timer Callback that attempts to select a new tentacle
	 */
	void planningCB(const ros::TimerEvent& event);

	/**
	 * Timer callback to send data to the platform at a constant rate
	 */
	void velUpdateCB(const ros::TimerEvent& event);

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for handling the SoftwareStop message
	 * @param message The message to process
	 */
	void stopCB(const aero_srr_msgs::SoftwareStopConstPtr& message);

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for processing new point cloud data
	 * @param message The aero_path_planning::OccupancyGridMsg message to process
	 *
	 * Takes the data from the PointCloud2 message, processes it into a new occupancy grid,
	 * and places it on the occupancy grid buffer for processing by the planner
	 */
	void pcCB(const aero_path_planning::OccupancyGridMsgConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief  Callback for handling Twist for manual/servo control
	 * @param message
	 */
	void manTwistCB(const geometry_msgs::TwistConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief  Handles changes to the robot's mission state
	 * @param message The current state of the robot
	 */
	void stateCB(const aero_srr_msgs::AeroStateConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief callback for handlining local LIDAR data
	 * @param message
	 */
	void lidarCB(const sensor_msgs::PointCloud2ConstPtr& message);

	/**
	 * @author Adam Panzica
	 * @brief Performs platform specific sending of velocity commands
	 * @param velocity The linear velocity in +x to follow
	 * @param radius The radius of curvature to follow
	 */
	void sendVelCom(double velocity, double radius);

	/**
	 * @author Adam Panzica
	 * @brief Sends out geometery_msgs::Twist messages to the platform
	 * @param x_dot The linear velocity in +x
	 * @param omega The angular velocity around +z
	 */
	void twist(double x_dot, double omega);


	void visualizeTentacle(int speed_set, int tentacle);

	/**
	 * @author Adam Panzica
	 * @brief  Loads parameters from the parameter server
	 */
	void loadParam();

	/**
	 * @author Adam Panzica
	 * @brief  Subscribes/Advertises topics with ROS
	 */
	void regTopic();

	/**
	 * @author Adam Panzica
	 * @brief  Regesters timers with ROS
	 */
	void regTimers();

	/**
	 * @author Adam Panzica
	 * @brief  Selects the best tentacle to follow
	 * @param [in]  current_vel  The current velocity of the robot, in m/s
	 * @param [in]  search_grid  The current local occupancy grid to search
	 * @param [out] speedset_idx The speed set index of the selected tentacle
	 * @param [out] tentacle_idx The tentacle index of the selected tentacle
	 * @return True if a tentacle was successfully selected, or false if there were no valid tentacles
	 */
	bool selectTentacle(const double& current_vel, const OccupancyGrid& search_grid, int& speedset_idx, int& tentacle_idx);

	/**
	 * @author Adam Panzica
	 * @return True if there is an obstruction within the bump-sensor radius of the robot
	 */
	bool bumpSwitch();

	/**
	 * @author Adam Panzica
	 * @brief Switches the local planner into manual control mode
	 */
	void setManualMode();

	/**
	 * @author Adam Panzica
	 * @brief Switches the local planner into tentacle control mode
	 */
	void setTentacleMode();

	/**
	 * @author Adam Panzica
	 * @brief  Sets the local planner to safe mode (will not move)
	 * @param [in] stop True to set safe mode, false to release it
	 */
	void setSafeMode(bool safe);

};

};

#endif /* LOCALPLANNER_H_ */
