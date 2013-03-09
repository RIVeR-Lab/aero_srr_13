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
#include<aero_drive_controller/VelocityCommandAction.h>
#include<boost/circular_buffer.hpp>
#include<aero_srr_msgs/SoftwareStop.h>
#include<geometry_msgs/Twist.h>
//********************** LOCAL  DEPENDANCIES **********************//
#include <aero_path_planning/OryxPathPlanning.h>
#include <aero_path_planning/OccupancyGridMsg.h>

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

	/**
	 * @author	Adam Panzica
	 * @brief	Does the actual work of implementing the Driving with Tentacles algorithm
	 *
	 */
	void doPlanning();
private:

	int		platform_;		///flat marking what platform we're running on
	bool	should_plan_;	///Flag for signaling if the local planner should be running

	double	goal_weight_;	///weighting factor to bias tentacle selection towards the goal point
	double	trav_weight_;	///weighting factor to bias tentacle selection away from previously traversed points
	double	diff_weight_;	///weighting factor to bias tentacle selection away from difficult terrain
	double  unkn_weight_;	///weighting factor to bias tentacle selection towards unknown terrain
	double	x_dim_;		///x dimension of the occupancy grid to use, in real units
	double	y_dim_;		///y dimension of the occupancy grid to use, in real units
	double	z_dim_;		///z dimension of the occupancy grid to use, in real units
	double	res_;		///resolution the occupancy grid to use, in real units per grid unit
	double	current_vel_;	///Current Velocity of the Platform
	double	current_rad_;	///Current Radius followed by the Platform
	double  set_vel_;	///The target velocity to set the robot to
	double  set_rad_;	///The target radius to set the robot to
	std::string	v_action_topic_;		///Actionlib topic name to send velocity commands over
	std::string pc_topic_;			///topic name of the ROS topic to receive new occupancy grid data over

	ros::NodeHandle nh_;	///Node handle for publishing/subscribing to topics
	ros::NodeHandle p_nh_;  ///Nodes handle to load private params
	ros::Subscriber 	pc_sub_;		///Subscriber to the ROS topic to receive new occupancy grid data over
	ros::Subscriber		stop_sub_;	///Subscriber to the ROS topic to receive the software stop message
	ros::Publisher		vel_pub_;	///Publisher for Twist messages to a platform that takes them
	ros::Publisher          tent_pub_;       ///Publisher for visualizing selected tentacles
	ros::Timer             vel_timer_;	///Timer that will send velocity updates to the platform at a constant rate

	aero_path_planning::Point	origin_;	///The origin to use for the occupancy grids
	TentacleGeneratorPtr tentacles_;	///Pointer to the tentacle generator which contains the tentacles to use for planning


	boost::circular_buffer<OccupancyGrid > occupancy_buffer_;	///Buffer to store received OccupancyGrid data

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
	 * Performs platform specific sending of velocity commands
	 * @param velocity The linear velocity in +x to follow
	 * @param radius The radius of curvature to follow
	 */
	void sendVelCom(double velocity, double radius);

	/**
	 * Sends out geometery_msgs::Twist messages to the platform
	 * @param x_dot The linear velocity in +x
	 * @param omega The angular velocity around +z
	 */
	void twist(double x_dot, double omega);


	void visualizeTentacle(int speed_set, int tentacle);

	void loadParam();

	void regTopic();

	void regTimers();

};

};

#endif /* LOCALPLANNER_H_ */
