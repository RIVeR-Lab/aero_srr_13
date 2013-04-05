#include "ros/ros.h"
#include "hd_driver/hd_motor_controller.h"
#include "aero_srr_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>

hd_driver::HDMotorController* controller;
ros::Publisher feedback_pub;
ros::Subscriber control_sub;
ros::Subscriber pause_sub;


void pauseCallback(const aero_srr_msgs::SoftwareStop::ConstPtr& msg){
}

void feedbackTimerCallback(const ros::TimerEvent& e){
}

#define define_and_get_param(type, var_name, param_name, default_value)	\
	type var_name(default_value);						\
	if(!ros::param::get(param_name, var_name))\
		ROS_WARN_STREAM("Parameter <"<<param_name<<"> not set. Using default value '"<<var_name<<"'")
	

int main(int argc, char **argv){
  ros::init(argc, argv, "hd_manager");
  
  ros::NodeHandle n;

  ROS_INFO("Initializing HD device");

  define_and_get_param(double, feedback_rate, "~feedback_rate", 1);
  define_and_get_param(std::string, port, "~port", "/dev/ttyACM0");
  define_and_get_param(std::string, control_topic, "~control_topic", "hd_control");
  define_and_get_param(std::string, info_topic, "~info_topic", "hd_info");
  define_and_get_param(std::string, pause_topic, "~pause_topic", "/pause");

  controller = new hd_driver::HDMotorController();

  controller->open(port);

  ROS_INFO("Initialization Complete");

  //control_sub = n.subscribe(control_topic, 1000, controlCallback);
  pause_sub = n.subscribe(pause_topic, 1000, pauseCallback);
  //feedback_pub = n.advertise<roboteq_driver::RoboteqGroupInfo>(info_topic, 1000);
  ros::Timer feedback_timer = n.createTimer(ros::Duration(1/feedback_rate), feedbackTimerCallback);
  
  ros::spin();

  delete controller;

  return 0;
}
