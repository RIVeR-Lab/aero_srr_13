/**
 * @file oryx_drive_controller.cpp
 *
 * @date Oct 1, 2012
 * @author Adam Panzica
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "VelocityControlServer.h"
#include "TranslateControlServer.h"
#include "OryxDriveControllerConfig.h"

class DriveController{
public:
	DriveController(std::string drive_velocity_topic,
					std::string drive_swerve_topic,
					std::string drive_capabilities_topic,
					double baseLength,
					double baseWidth){
		ROS_INFO("Starting Up Low Level Drive Controller");
		//TODO actually set up correct message publishing data
		//Set up publisher to the DriveManager wheel velocity topic
		this->velocity_pub = nh.advertise<std_msgs::String>(drive_velocity_topic.c_str(),2);
		//Set up publisher to the DriveManager swerve control topic
		this->swerve_pub = nh.advertise<std_msgs::String>(drive_swerve_topic.c_str(),2);
		//set up dimensional parameters
		this->baseLength = baseLength;
		this->baseWidth  = baseWidth;
		ROS_DEBUG("Got a platform size of <L=%f,W=%f>",baseLength, baseWidth);
	}
private:
	double baseLength;
	double baseWidth;
	ros::NodeHandle nh;
	ros::Publisher velocity_pub;
	ros::Publisher swerve_pub;
};

///Used by ROS to create the node.
int main(int argc, char** argv)
{
	std::string v_action_topic;	///String containing the topic name for velocity commands
	std::string t_action_topic;	///String containing the topic name for translate commands
	std::string v_drive_topic;	///String containing the topic name for sending wheel velocities to the DriveManager node
	std::string s_drive_topic;	///String containing the topic name for sending swerve positions to the DriveManager node
	std::string c_drive_topic;	///String containing the topic name for polling the DriveManager node on its capabilities
	double baseWidth;			///width of the platform
	double baseLength;			///length of the platform

	//Initialize the node
	ros::init(argc, argv, "oryx_drive_controller");
	//Get a private node handle to parse command line arguments
	ros::NodeHandle param_nh("~");
	ROS_INFO("Oryx Drive Controller Version: %d.%d.%d", oryx_drive_controller_VERSION_MAJOR, oryx_drive_controller_VERSION_MINOR, oryx_drive_controller_VERSION_BUILD);
	//Just a check to make sure the usage was correct
	if(argc < 1){
		ROS_INFO("Usage: oryx_drive_controller velocity_command_topic translate_command_topic drive_velocity_topic drive_swerve_topic drive_capability_topic");
		return 1;
	}else{
		//Grab the topic parameters
		param_nh.getParam("velocity_command_topic",		v_action_topic);
		param_nh.getParam("translate_command_topic",	t_action_topic);
		param_nh.getParam("drive_velocity_topic",		v_drive_topic);
		param_nh.getParam("drive_swerve_topic",			s_drive_topic);
		param_nh.getParam("drive_capability_topic",		c_drive_topic);
		param_nh.getParam("base_length",				baseLength);
		param_nh.getParam("base_width",					baseWidth);
	}

	//Print out recieved topics
	ROS_DEBUG("Got Velocity Action Topic Name: <%s>",		v_action_topic.c_str());
	ROS_DEBUG("Got Translate Action Topic Name: <%s>",		t_action_topic.c_str());
	ROS_DEBUG("Got DriveManager Velocity Topic Name: <%s>",	v_drive_topic.c_str());
	ROS_DEBUG("Got DriveManager Swerve Topic Name: <%s>",	s_drive_topic.c_str());
	ROS_DEBUG("Got DriveManager Capability Topic Name: <%s>",	c_drive_topic.c_str());

	//Spawn the servers
	ROS_INFO("Starting Up Oryx Drive Controller...");
	VelocityControlServer v_server(v_action_topic);
	TranslateControlServer t_server(t_action_topic);
	DriveController d_controller(v_drive_topic, s_drive_topic, c_drive_topic, baseLength, baseWidth);
	ROS_INFO("Oryx Drive Controller Running!");
	ros::spin();
	return 0;
}
