/**
 * @file oryx_drive_controller.cpp
 *
 * @date Oct 1, 2012
 * @author Adam Panzica
 */

#include <ros/ros.h>
#include "VelocityControlServer.h"
#include "TranslateControlServer.h"
#include "OryxDriveControllerConfig.h"


//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
	std::string v_action_topic;
	std::string t_action_topic;

	ros::init(argc, argv, "oryx_drive_controller");
	//Get a private node handle to parse command line arguments
	ros::NodeHandle param_nh("~");
	ROS_INFO("Oryx Drive Controller Version: %d.%d.%d", oryx_drive_controller_VERSION_MAJOR, oryx_drive_controller_VERSION_MINOR, oryx_drive_controller_VERSION_BUILD);
	//Just a check to make sure the usage was correct
	if(argc < 1){
		ROS_INFO("Usage: oryx_drive_controller _v_action_topic:=string _t_action_topic");
		return 1;
	}else{
		param_nh.getParam("v_action_topic", v_action_topic);
		param_nh.getParam("t_action_topic", t_action_topic);
	}

	ROS_INFO("Got Velocity Action Topic Name: <%s>", v_action_topic.c_str());
	ROS_INFO("Got Translate Action Topic Name: <%s>", t_action_topic.c_str());
	//Spawn the servers
	ROS_INFO("Starting Up Oryx Drive Controller...");
	VelocityControlServer v_server(v_action_topic);
	TranslateControlServer t_server(t_action_topic);
	ROS_INFO("Oryx Drive Controller Running!");
	ros::spin();
	return 0;
}
