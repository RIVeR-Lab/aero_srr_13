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

	void executeCB(const oryx_drive_controller::VelocityCommandGoalConstPtr& goal);
private:
	ros::NodeHandle n;
	actionlib::SimpleActionServer<oryx_drive_controller::VelocityCommandAction>	as;
	oryx_drive_controller::VelocityCommandFeedback	feedback;
	oryx_drive_controller::VelocityCommandResult	result;
	std::string 									action_name;
};

/*VelocityControlServer::VelocityControlServer(std::string action_name):
as(n, action_name, boost::bind(&VelocityControlServer::executeCB, this, _1), false),
action_name(action_name){
	ROS_INFO("Starting Up Velocity Command Server on <%s>", this->action_name.c_str());
}

void VelocityControlServer::executeCB(const oryx_drive_controller::VelocityCommandGoalConstPtr& goal){
	ROS_INFO("Got Goal <R=%f, V=%f> on <%s>", goal->radius,goal->velocity, this->action_name.c_str());
}*/

#endif /* VELOCITYCONTROLSERVER_H_ */
