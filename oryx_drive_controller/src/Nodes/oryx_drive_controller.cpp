/**
 * @file oryx_drive_controller.cpp
 *
 * @date Oct 1, 2012
 * @author Adam Panzica
 */

#include <ros/ros.h>
#include "VelocityControlServer.h"


//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
	std::string action_topic;
	ros::init(argc, argv, "oryx_drive_controller");
	//Just a check to make sure the usage was correct
	if(argc != 2){
		ROS_INFO("Usage: oryx_drive_controller velocity_action_topic");
		return 1;
	}else{
		action_topic(argv[1]);
	}
	ROS_INFO("Got Action Topic Name: <%s>", action_topic.c_str());
	//Spawn the server
	ROS_INFO("Starting Up Oryx Drive Controller...");
	//VelocityControlServer server(action_topic);

	ros::spin();
	return 0;
}
