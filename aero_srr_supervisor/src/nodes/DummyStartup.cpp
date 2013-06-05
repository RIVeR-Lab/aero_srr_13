/**
 * @file   DummyStartup.cpp
 *
 * @date   Mar 22, 2013
 * @author Adam Panzica
 * @brief  prevents robot from running
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
#include <aero_srr_msgs/StateTransitionRequest.h>
#include<robot_base_msgs/SoftwareStop.h>
//************ LOCAL DEPENDANCIES ****************//

//***********  NAMESPACES     ****************//

bool got_free = false;
ros::ServiceClient client;
ros::Subscriber    sub;

void requestStateTransition(aero_srr_msgs::AeroState& requested_state)
{
	aero_srr_msgs::StateTransitionRequestRequest request;
	aero_srr_msgs::StateTransitionRequestResponse response;
	request.requested_state = requested_state;
	if(client.call(request, response))
	{
		if(response.success)
		{
			ROS_INFO_STREAM("Dummmy Node Succesfully Moved to "<<(int)requested_state.state<<" !");
			got_free = true;
		}
		else
		{
			ROS_ERROR_STREAM("Dummmy Node could not transition to "<<(int)requested_state.state<<": "<<response.error_message);
		}
	}
	else
	{
		ROS_ERROR_STREAM("Something went wrong with  Dummmy Node trying to request state change!");
	}
}


void stopCB(const robot_base_msgs::SoftwareStopConstPtr& message){
	//Need to flip as message is true when should stop
	if(message->stop)
	{
		got_free = false;
	}
	else
	{
		aero_srr_msgs::AeroState req;
		req.state = aero_srr_msgs::AeroState::SEARCH;
		requestStateTransition(req);
	}
	ROS_WARN("Dummy Starrtup Node Got Software Stop [%s]: %s",(message->stop)?"Stop":"Go", message->message.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dummy_startup");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");
	client = nh.serviceClient<aero_srr_msgs::StateTransitionRequest>("/aero/supervisor/state_transition_request");
	sub= nh.subscribe("aero/software_stop", 1, stopCB);
	
	while(!got_free&&ros::ok())
	{
		ros::spinOnce();
	}
	return 1;
}

