/**
 * @file VelocityControlServer.h
 * @date Oct 1, 2012
 * @author Adam Panzica
 */

#ifndef VELOCITYCONTROLSERVER_H_
#define VELOCITYCONTROLSERVER_H_

#include <ros/ros.h>
#include <oryx_drive_controller/VelocityCommandAction.h>
#include <oryx_drive_controller/VelocityArc.h>
#include <actionlib/server/simple_action_server.h>


class VelocityControlServer {
public:
	/**
	 * Constructor for creating a new VelocityControlServer
	 * @param action_name			String representing the name of the action topic to communicate on
	 * @param ctrl_velocity_topic	String representing the intra-processes topic name to communicate velocity-arc with the controller
	 */
	VelocityControlServer(std::string action_name, std::string ctrl_velocity_topic);
	virtual ~VelocityControlServer();
private:
	/**
	 * Callback for executing a new velocity command
	 * @param goal the goal message to process
	 */
	void executeCB(const oryx_drive_controller::VelocityCommandGoalConstPtr& goal);

	ros::NodeHandle n;			///Node handle for getting parameters
	ros::Publisher vel_pub;		///Publisher for sending velocity-arc messages to controller
	actionlib::SimpleActionServer<oryx_drive_controller::VelocityCommandAction>	as;	///Actionlib server for processing velocity commands
	oryx_drive_controller::VelocityCommandFeedback	feedback;						///Feedback message for actionlib server
	oryx_drive_controller::VelocityCommandResult	result;							///Result message for actionlib server
	std::string 									action_name;					///Action topic name
};

#endif /* VELOCITYCONTROLSERVER_H_ */
