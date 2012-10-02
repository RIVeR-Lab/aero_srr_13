/*
 * TranslateControlServer.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: parallels
 */

#include "TranslateControlServer.h"

TranslateControlServer::TranslateControlServer(std::string action_name):
as(n, action_name, boost::bind(&TranslateControlServer::executeCB, this, _1), false),
action_name(action_name) {
	ROS_INFO("Starting Up Traverse Command Server on <%s>", this->action_name.c_str());

}

TranslateControlServer::~TranslateControlServer() {
}

void TranslateControlServer::executeCB(const oryx_drive_controller::TranslateCommandGoalConstPtr& goal){
	ROS_INFO("Got Goal <X_v=%f, Y-v=%f> on <%s>", goal->x_velocity,goal->y_velocity, this->action_name.c_str());
}
