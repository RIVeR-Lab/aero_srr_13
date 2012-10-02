/**
 * @file VelocityControlServer.cpp
 *
 *  @date Oct 1, 2012
 *  @author Adam Panzica
 */

#include "VelocityControlServer.h"

VelocityControlServer::VelocityControlServer(std::string action_name):
as(n, action_name, boost::bind(&VelocityControlServer::executeCB, this, _1), false),
action_name(action_name){
	ROS_INFO("Starting Up Velocity Command Server on <%s>", this->action_name.c_str());
}

void VelocityControlServer::executeCB(const oryx_drive_controller::VelocityCommandGoalConstPtr& goal){
	ROS_INFO("Got Goal <R=%f, V=%f> on <%s>", goal->radius,goal->velocity, this->action_name.c_str());
}
