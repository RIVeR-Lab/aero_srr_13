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
	this->as.start();
	//Advertise publishing to the controller's topics
	this->vel_pub = this->n.advertise<aero_drive_controller::VelocityArc>(ctrl_velocity_topic, 2);
}

VelocityControlServer::~VelocityControlServer(){
	ROS_INFO("Shutting Down Server...");
	as.shutdown();
}
/**
 * Currently, all this does is pipe the received goal directly to the ArcDriveController. At some point it should do some
 * more advanced processing and feedback development
 */
void VelocityControlServer::executeCB(const aero_drive_controller::VelocityCommandGoalConstPtr& goal){
	ROS_INFO("Got Goal <R=%f, V=%f> on <%s>", goal->radius,goal->velocity, this->action_name.c_str());
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
		aero_drive_controller::VelocityArcPtr msg(new aero_drive_controller::VelocityArc);

		//Build the message
		msg->radius = goal->radius;
		msg->velocity = goal->velocity;

		//Publish the message
		this->vel_pub.publish(msg);

		this->result.capable = true;
		this->result.success = true;
		success = true;
	}
	//set success
	if(success){
		this->as.setSucceeded(this->result, "I successfully set the new Velocity Arc goal");
	}
}
