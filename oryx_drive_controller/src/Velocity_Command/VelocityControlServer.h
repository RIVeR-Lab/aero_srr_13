/**
 * @file VelocityControlServer.h
 * @date Oct 1, 2012
 * @author Adam Panzica
 */

#ifndef VELOCITYCONTROLSERVER_H_
#define VELOCITYCONTROLSERVER_H_

#include <ros/ros.h>
#include <oryx_drive_controller/VelocityCommandAction.h>
#include <actionlib/server/simple_action_server.h>

class VelocityControlServer {
public:
	VelocityControlServer(std::string action_name);
	virtual ~VelocityControlServer();
private:
	ros::NodeHandle n;
	actionlib::SimpleActionServer<oryx_drive_controller::VelocityCommandAction>	as;
	oryx_drive_controller::VelocityCommandFeedback	feedback;
	oryx_drive_controller::VelocityCommandResult	result;
	std::string 				action_name;
};

#endif /* VELOCITYCONTROLSERVER_H_ */
