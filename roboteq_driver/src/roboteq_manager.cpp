#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"




void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
}

int main(int argc, char **argv){
  ros::init(argc, argv, "roboteq_manager");
  
  ros::NodeHandle n;

  ROS_INFO("Initializing RoboteqDevice");


  if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
	  cout<<"failed --> "<<status<<endl;
  else
	  cout<<"succeeded."<<endl;
  

  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, twistCallback);

  ros::spin();

  return 0;
}

/*

	//Wait 10 ms before sending another command to device
	sleepms(10);

	int result;
	cout<<"- GetConfig(_DINA, 1)...";
	if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"returned --> "<<result<<endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	cout<<"- GetValue(_ANAIN, 1)...";
	if((status = device.GetValue(_ANAIN, 1, result)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"returned --> "<<result<<endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	cout<<"- SetCommand(_GO, 1, 1)...";
	if((status = device.SetCommand(_GO, 1, 1)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"succeeded."<<endl;

*/
