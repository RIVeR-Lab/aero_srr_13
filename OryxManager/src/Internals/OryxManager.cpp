#include "OryxManager.h"
#include "std_msgs/String.h"
#include <OryxManager/OryxManagerConfig.h>

ros::Publisher heartbeatPublisher;
bool remoteConnection=false;
int remoteConnectionAttempts=0;

void heartbeatCallback(const ros::TimerEvent&){
	if (remoteConnection)remoteConnectionAttempts++;
	else remoteConnectionAttempts=0;
	if(remoteConnectionAttempts <10){
		std_msgs::String msg;
		msg.data= "Okay";
		heartbeatPublisher.publish(msg);
	}
}

void batteryCallback(const OryxMessages::Battery::ConstPtr& msg){
	if(msg->voltage <2.2) ROS_WARN_STREAM("Battery Node " << msg->node << " is low. Shut off soon." );
	if(msg->voltage < 2.1 && msg->voltage >1.0){
		ROS_ERROR_STREAM("Battery Node " << msg->node << " is at " << msg->voltage <<". Shutting Down...");
		std_msgs::String msg;
		msg.data= "Shut Down";
		heartbeatPublisher.publish(msg);
		sleep(10);
		ROS_ERROR_STREAM("Shutting Down Now");
		system("sudo halt now");
	}

}

void temperatureCallback(const OryxMessages::Temperature::ConstPtr& msg){
	if(msg->temperature > msg->danger_Temp){
		ROS_WARN_STREAM("NODE " << msg->temperature_node << " TEMPERATURE IS DANGEROUSLY HIGH");
	}
	else if(msg->temperature > msg->warning_Temp){
		ROS_WARN_STREAM("NODE " << msg->temperature_node << " TEMPERATURE IS APPROACHING DANGEROUS TEMPERATURES");
	}
}

void reconfigureCallback(OryxManager::OryxManagerConfig &config, uint32_t level){
	remoteConnection=config.Remote_Connection;
}

void remoteConnectionCallback(const std_msgs::String::ConstPtr& msg){
	remoteConnectionAttempts=0;
}

int main(int argc, char** argv){
	//Intialize ROS node with name OryxManager
	ros::init(argc, argv, "OryxManager");
	ros::NodeHandle nh;

	//Create publisher that lets others know the robot is okay
	heartbeatPublisher 	= nh.advertise<std_msgs::String>("/Heartbeat",1);

	//Publish a heartbeat at 10Hz
	ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(.1),heartbeatCallback);

	//Listen to the temperatures, run temperatureCallback when one is heard
	ros::Subscriber tempSub = nh.subscribe("Temps",20,temperatureCallback);

	//Listen to voltages, run batteryCallback when one is heard
	ros::Subscriber batSub = nh.subscribe("Battery",10,batteryCallback);

	//Listen for remote computers. If remote connection is lost, rover will stop moving
	ros::Subscriber remoteConnectionHeartbeat
		= nh.subscribe("/RemoteConnection",1,remoteConnectionCallback);

	//Begin dynamic_reconfigure server
	dynamic_reconfigure::Server<OryxManager::OryxManagerConfig> server;
	dynamic_reconfigure::Server<OryxManager::OryxManagerConfig>::CallbackType f;

	//Call reconfigureCallback when new dynamic reconfigure update received
	f = boost::bind(reconfigureCallback, _1 ,_2);
	server.setCallback(f);

	//Start Multithreading callback handler
	ros::AsyncSpinner spinner(4);
	spinner.start();

	//Wait for ROS to shutdown
	ros::waitForShutdown();
}
