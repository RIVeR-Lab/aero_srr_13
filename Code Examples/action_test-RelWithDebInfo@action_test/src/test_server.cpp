/*
 * test_server.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: parallels
 */



#include <ros/ros.h>
#include <action_test/TestAction.h>
#include <actionlib/server/simple_action_server.h>

//Class for containing the server
class ActionTestServer{
public:
	ActionTestServer(std::string name):
		//Set up the action server. It needs a reference to the node handle,
		//Is subscribed to topic "test_action", and registers a callback for excicuting

		as(n, "test_action", boost::bind(&ActionTestServer::executeCB, this, _1), false),
		action_name(name)
	{
		//Register Callbacks. Need to use boost::bind because it is inside a class instance
		as.registerPreemptCallback(boost::bind(&ActionTestServer::preemptCB, this));
		//Start the server
		as.start();
	}

	//Callback for handling preemption. Reset your helpers here.
	//Note that you still have to check for preemption in your work method to break it off
	void preemptCB(){
		ROS_INFO("%s got preempted!", action_name.c_str());
		result.result = progress;
		as.setPreempted(result, "I got Preempted!");
	}

	//Callback for processing a goal
	void executeCB(const action_test::TestGoalConstPtr& goal)
	{
		//If the server has been killed, don't process
		if(!as.isActive()||as.isPreemptRequested()) return;

		//Run the processing at 5Hz
		ros::Rate rate(5);
		//Setup some local variables
		bool success	= true;

		ROS_INFO("%s is Processing Goal: %d", action_name.c_str(), goal->request);

		//Process the request
		for (progress = 0; progress < goal->request; progress++) {

			//Check for Ros kill
			if(!ros::ok()){
				success = false;
				ROS_INFO("%s Shutting Down", action_name.c_str());
				break;
			}

			//If the server has been killed/preempted, stop processing
			if(!as.isActive()||as.isPreemptRequested()){
				return;
			}

			//Publish a status update
			ROS_INFO("I'm getting to goal, %d/%d", feedback.progress, goal->request);
			feedback.progress = progress;
			as.publishFeedback(feedback);

			//Sleep for rate time
			rate.sleep();
		}

		//Publish the result if the goal wasn't preempted
		if(success){
			ROS_INFO("%s Succeeded at Getting to Goal: %d", action_name.c_str(), goal->request);

			as.setSucceeded(result);
		} else {
			result.result = progress;
			as.setAborted(result,"I Failed!");
		}

	}

protected:
	ros::NodeHandle n;
	actionlib::SimpleActionServer<action_test::TestAction>	as;
	action_test::TestFeedback	feedback;
	action_test::TestResult		result;
	std::string 				action_name;
	int 						goal;
	int							progress;

};

//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_action_server");

	//Just a check to make sure the usage was correct
	if(argc != 1){
		ROS_INFO("Usage: test_server");
		return 1;
	}
	//Spawn the server
	ActionTestServer server(ros::this_node::getName());

	ros::spin();
	return 0;
}
