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
	//Start up the command server
	as.start();
	//Advertise to intra-process publisher
	this->trans_pub = this->n.advertise<oryx_drive_controller::VelocityTranslate>(ctrl_translate_topic, 2);

}

TranslateControlServer::~TranslateControlServer() {
}

void TranslateControlServer::executeCB(const oryx_drive_controller::TranslateCommandGoalConstPtr& goal){
	ROS_INFO("Got Goal <X_v=%f, Y-v=%f> on <%s>", goal->x_velocity,goal->y_velocity, this->action_name.c_str());

	bool success;
	// check that preempt has not been requested by the client
	if (as.isPreemptRequested() || !ros::ok())
	{
		ROS_INFO("%s: Preempted", action_name.c_str());
		// set the action state to preempted
		as.setPreempted();
		success = false;
	}

	else{
			//Create a new message to send to the controller
			oryx_drive_controller::VelocityTranslatePtr msg(new oryx_drive_controller::VelocityTranslate);

			//Build the message
			msg->x_velocity = goal->x_velocity;
			msg->y_velocity = goal->y_velocity;

			//Publish the message
			this->trans_pub.publish(msg);

			this->result.capable = true;
			this->result.success = true;
			success = true;
		}
		//set success
		if(success){
			this->as.setSucceeded(this->result, "I successfully set the new Velocity Translate goal");
	}
}
