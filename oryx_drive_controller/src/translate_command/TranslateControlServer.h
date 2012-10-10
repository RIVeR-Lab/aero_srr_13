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
#include <oryx_drive_controller/VelocityTranslate.h>
#include <actionlib/server/simple_action_server.h>

class TranslateControlServer {
public:
	/**
	 *
	 * @param action_name			String representing the name of the action topic to communicate on
	 * @param ctrl_translate_topic	String representing the intra-processes topic name to communicate velocity-translate with the controller
	 */
	TranslateControlServer(std::string action_name,  std::string ctrl_translate_topic);
	virtual ~TranslateControlServer();

private:

	void executeCB(const oryx_drive_controller::TranslateCommandGoalConstPtr& goal);

	ros::NodeHandle n;
	ros::Publisher trans_pub;	///Publisher for sending velocity-translate messages to controller
	actionlib::SimpleActionServer<oryx_drive_controller::TranslateCommandAction>	as;
	oryx_drive_controller::TranslateCommandFeedback	feedback;
	oryx_drive_controller::TranslateCommandResult	result;
	std::string 									action_name;
};

#endif
