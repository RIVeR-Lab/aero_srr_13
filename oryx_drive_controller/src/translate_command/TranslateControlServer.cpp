/*
 * TranslateControlServer.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: parallels
 */

#include "TranslateControlServer.h"

TranslateControlServer::TranslateControlServer(std::string action_name, std::string ctrl_translate_topic):
as(n, action_name, boost::bind(&TranslateControlServer::executeCB, this, _1), false),
action_name(action_name) {
	ROS_INFO("Starting Up Traverse Command Server on <%s>", this->action_name.c_str());

	//Advertise to intra-process publisher
	this->trans_pub = this->n.advertise<oryx_drive_controller::VelocityTranslate>(ctrl_translate_topic, 2);

}

TranslateControlServer::~TranslateControlServer() {
}

void TranslateControlServer::executeCB(const oryx_drive_controller::TranslateCommandGoalConstPtr& goal){
	ROS_INFO("Got Goal <X_v=%f, Y-v=%f> on <%s>", goal->x_velocity,goal->y_velocity, this->action_name.c_str());
}
