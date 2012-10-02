/**
 * @file TranslateControlServer.h
 *
 * @date Oct 2, 2012
 * @author Adam Panzica
 */

#ifndef TRANSLATECONTROLSERVER_H_
#define TRANSLATECONTROLSERVER_H_

#include <ros/ros.h>
#include <oryx_drive_controller/TranslateCommandAction.h>
#include <actionlib/server/simple_action_server.h>

class TranslateControlServer {
public:
	TranslateControlServer(std::string action_name);
	virtual ~TranslateControlServer();

	void executeCB(const oryx_drive_controller::TranslateCommandGoalConstPtr& goal);
private:
	ros::NodeHandle n;
	actionlib::SimpleActionServer<oryx_drive_controller::TranslateCommandAction>	as;
	oryx_drive_controller::TranslateCommandFeedback	feedback;
	oryx_drive_controller::TranslateCommandResult	result;
	std::string 									action_name;
};

#endif
