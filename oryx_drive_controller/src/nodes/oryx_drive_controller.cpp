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
#include "ArcDriveController.h"


/**
 * Used by ROS to create the node
 * @param argc number of private arguments passed to the node
 * @param argv pointers to the arguments
 * The oryx_drive_controller node looks for the following private parameters from the parameter server on startup:
 * |	Parameter			|	Type	|			Value																|
 * |	:---:				|	:---:	|			:---:																|
 * | velocity_command_topic	|	String	| Action Topic name to receive arc commands on									|
 * | translate_command_topic|	String	| Action Topic name to receive translate commands on							|
 * | drive_velocity_topic	|	String	| Topic name to send wheel velocity messages to base platform					|
 * | drive_swerve_topic		|	String	| Topic name to send swerve position messages to base platform					|
 * | drive_capability_topic	|	String	| Topic name to request capability messages from base							|
 * | base_length			|	Double	| The length of the base platform from front wheel center to rear wheel center	|
 * | base_width				|	Double	| The width of the base platform from left wheel center to right wheel center	|
 */
int main(int argc, char** argv)
{
	std::string v_action_topic("velocity_command_topic");	///String containing the topic name for velocity commands
	std::string t_action_topic("translate_command_topic");	///String containing the topic name for translate commands
	std::string v_drive_topic("drive_velocity_topic");		///String containing the topic name for sending wheel velocities to the DriveManager node
	std::string s_drive_topic("drive_swerve_topic");		///String containing the topic name for sending swerve positions to the DriveManager node
	std::string c_drive_topic("drive_capability_topic");	///String containing the topic name for polling the DriveManager node on its capabilities
	std::string ip_v_arc_topic("ip_v_arc_topic");			///String containing the topic name for intra-processes VelocityArc message publishing
	std::string ip_t_arc_topic("ip_t_arc_topic");			///String containing the topic name for intra-processes VelocityTranslate message publishing
	std::string b_length("base_length");		///String containing the parameter name for baseLength
	std::string b_width("base_width");			///String containing the parameter name for baseWidth
	double baseWidth = .36;						///width of the platform (the default value is overwritten if the appropriate param is set)
	double baseLength = .39;					///length of the platform (the default value is overwritten if the appropriate param is set)

	//Initialize the node
	ros::init(argc, argv, "oryx_drive_controller");
	//Get a private node handle to parse command line arguments
	ros::NodeHandle param_nh("~");
	ROS_INFO("Oryx Drive Controller Version: %u.%u.%u", oryx_drive_controller_VERSION_MAJOR, oryx_drive_controller_VERSION_MINOR, oryx_drive_controller_VERSION_BUILD);
	//Just a check to make sure the usage was correct
	if(argc < 1){
		ROS_INFO("Usage: oryx_drive_controller velocity_command_topic translate_command_topic drive_velocity_topic drive_swerve_topic drive_capability_topic");
		return 1;
	}else{
		//Grab the topic parameters, print warnings if using default values
		if(!param_nh.getParam(v_action_topic,	v_action_topic))ROS_WARN("Parameter <%s> Not Set. Using Default Velocity Command Action Topic <%s>!", v_action_topic.c_str(), v_action_topic.c_str());
		if(!param_nh.getParam(t_action_topic,	t_action_topic))ROS_WARN("Parameter <%s> Not Set. Using Default Translate Command Action Topic <%s>!", t_action_topic.c_str(), t_action_topic.c_str());
		if(!param_nh.getParam(v_drive_topic,	v_drive_topic))	ROS_WARN("Parameter <%s> Not Set. Using Default Wheel Velocity Topic <%s>!", v_drive_topic.c_str(), v_drive_topic.c_str());
		if(!param_nh.getParam(s_drive_topic,	s_drive_topic))	ROS_WARN("Parameter <%s> Not Set. Using Default Swerve Position Topic <%s>!", s_drive_topic.c_str(), s_drive_topic.c_str());
		if(!param_nh.getParam(c_drive_topic,	c_drive_topic))	ROS_WARN("Parameter <%s> Not Set. Using Default Platform Capability Topic <%s>!", c_drive_topic.c_str(), c_drive_topic.c_str());
		if(!param_nh.getParam(b_length,			baseLength))	ROS_WARN("Parameter <%s> Not Set. Using Default Base Length <%f>!", b_length.c_str(), baseLength);
		if(!param_nh.getParam(b_width,			baseWidth))		ROS_WARN("Parameter <%s> Not Set. Using Default Base Length <%f>!", b_width.c_str(), baseWidth);
	}

	//Print out recieved topics
	ROS_DEBUG("Got Velocity Action Topic Name: <%s>",		v_action_topic.c_str());
	ROS_DEBUG("Got Translate Action Topic Name: <%s>",		t_action_topic.c_str());
	ROS_DEBUG("Got DriveManager Velocity Topic Name: <%s>",	v_drive_topic.c_str());
	ROS_DEBUG("Got DriveManager Swerve Topic Name: <%s>",	s_drive_topic.c_str());
	ROS_DEBUG("Got DriveManager Capability Topic Name: <%s>",	c_drive_topic.c_str());

	//Spawn the servers. They will act as nodelits to each other, and use intra-process publishing
	ROS_INFO("Starting Up Oryx Drive Controller...");
	VelocityControlServer v_server(v_action_topic, ip_v_arc_topic);
	TranslateControlServer t_server(t_action_topic, ip_t_arc_topic);
	ArcDriveController lld_controller(v_drive_topic, s_drive_topic, c_drive_topic, ip_v_arc_topic, ip_t_arc_topic, baseLength, baseWidth);

	/*	ROS_INFO("I'm Testing Stuff Now...");
	ROS_INFO("Testing Steer With Velocity = 1m/s, Radius = 5m");
	lld_controller.setCanSwerve(false);
	lld_controller.drive(1,5);
	lld_controller.setCanSwerve(true);
	lld_controller.drive(1,5);
	ROS_INFO("Testing Translate With X_V=1, Y_V=1");
	lld_controller.translate(1,1);*/

	ROS_INFO("Oryx Drive Controller Running!");
	ros::spin();
	return 0;
}
