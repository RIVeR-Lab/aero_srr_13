/*
 * test_client.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: parallels
 */



#include <ros/ros.h>
#include <action_test/TestAction.h>
#include <actionlib/client/simple_action_client.h>



//class containing the client
class ActionTestClient{
public:
	ActionTestClient(std::string name):
		//Set up the client. It's publishing to topic "test_action", and is set to auto-spin
		ac("test_action", true),
		//Stores the name
		action_name(name){
		//Get connection to a server
		ROS_INFO("%s Waiting For Server...", action_name.c_str());
		//Wait for the connection to be valid
		ac.waitForServer();
		ROS_INFO("%s Got a Server...", action_name.c_str());
	}
	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const action_test::TestResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Result: %d", result->result);
		ros::shutdown();
	};

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active...");
	};

	// Called every time feedback is received for the goal
	void feedbackCb(const action_test::TestFeedbackConstPtr& feedback)
	{
		ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->progress);
	};

	//Send a goal to the server
	void send(int goal){
		action_test::TestGoal newGoal;
		newGoal.request = goal;
		//Once again, have to used boost::bind because you are inside a class
		ac.sendGoal(newGoal, boost::bind(&ActionTestClient::doneCb, this, _1, _2),
							 boost::bind(&ActionTestClient::activeCb, this),
							 boost::bind(&ActionTestClient::feedbackCb, this, _1));
	}

private:
	actionlib::SimpleActionClient<action_test::TestAction> ac;
	std::string action_name;
};






//Used by ROS to actually create the node. Could theoretically spawn more than one client
int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_action_client");
	int sampleGoal = 100;
	//Usage check to make sure the client is being used properly
	if(argc != 2){
		ROS_INFO("Usage: test_client <goal>");
		return 1;
	}else{
		sampleGoal = atoll(argv[1]);
	}

	//Initialize the client
	ActionTestClient client(ros::this_node::getName());

	//Send the goal to the server
	client.send(sampleGoal);
	ROS_INFO("Sent Goal %d To Server...", sampleGoal);

	ros::spin();
	return 0;
}
