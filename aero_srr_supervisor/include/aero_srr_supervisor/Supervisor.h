/**
 * @file   Supervisor.h
 *
 * @date   Mar 22, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef SUPERVISOR_H_
#define SUPERVISOR_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
#include <tf/transform_listener.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_srr_supervisor/SupervisorUtilities.h>
#include <aero_srr_supervisor/SetControlMode.h>
//***********    NAMESPACES     ****************//
namespace aero_srr_supervisor
{

class Supervisor
{
public:
	Supervisor(ros::NodeHandle& nh, ros::NodeHandle& p_nh);

private:

	/**
	 * @author Adam Panzica
	 * @brief  Initialization function, Loads parameters from the ROS param server
	 */
	void loadParams();
	/**
	 * @author Adam Panzica
	 * @brief  Initialization function, Registers topics with the ROS system
	 */
	void registerTopics();
	/**
	 * @author Adam Panzica
	 * @brief  Initialization function, Registers timers with the ROS system
	 */
	void registerTimers();

	/**
	 * @author Adam Panzica
	 * @brief  Callback for processing SetControlMode messages
	 * @param message
	 */
	void setCtrlMdCB(const aero_srr_supervisor::SetControlModeConstPtr& message);

	std::string ctrlmd_topic_;

	ros::NodeHandle nh_;
	ros::NodeHandle p_nh_;
	ros::Subscriber ctrlmd_sub_;

	tf::TransformListener transformer_;
};

};

#endif /* SUPERVISOR_H_ */
