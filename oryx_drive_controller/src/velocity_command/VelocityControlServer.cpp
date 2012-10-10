/**
 * @file VelocityControlServer.cpp
 *
 *  @date Oct 1, 2012
 *  @author Adam Panzica
 */

#include "VelocityControlServer.h"

VelocityControlServer::VelocityControlServer(std::string action_name,std::string ctrl_velocity_topic):
as(n, action_name, boost::bind(&VelocityControlServer::executeCB, this, _1), false),
action_name(action_name){
	ROS_INFO("Starting Up Velocity Command Server on <%s>", this->action_name.c_str());

	//Advertise publishing to the controller's topics
	this->vel_pub = this->n.advertise<oryx_drive_controller::VelocityArc>(ctrl_velocity_topic, 2);
}

void VelocityControlServer::executeCB(const oryx_drive_controller::VelocityCommandGoalConstPtr& goal){
	ROS_INFO("Got Goal <R=%f, V=%f> on <%s>", goal->radius,goal->velocity, this->action_name.c_str());

	//Create a new message to send to the controller
	oryx_drive_controller::VelocityArcPtr msg(new oryx_drive_controller::VelocityArc);

	//Build the message
	msg->radius = goal->radius;
	msg->velocity = goal->velocity;

	//Publish the message
	this->vel_pub.publish(msg);

	this->result.capable = true;
	this->result.success = true;
	//set success
	this->as.setSucceeded(this->result, "I successfully set the new Velocity Arc goal");
}
