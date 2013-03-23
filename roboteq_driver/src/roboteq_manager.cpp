#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roboteq_driver/roboteq_motor_controller.h"
#include "roboteq_driver/RoboteqFeedback.h"
#include "aero_srr_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

static boost::mutex controller_mutex;
roboteq_driver::RoboteqMotorController* controller;
ros::Publisher feedback_pub;
ros::Subscriber twist_sub;
ros::Subscriber pause_sub;
bool is_paused = false;


void drive(double u, double w){
	//TODO do inverse kinematics here...
	double left_speed = -u+w;
	double right_speed = u+w;
	{
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		if(is_paused)
			controller->setSpeed(0, 0);
		else
			controller->setSpeed(left_speed, right_speed);
	}
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
	drive(msg->linear.x, msg->angular.z);
}
void pauseCallback(const aero_srr_msgs::SoftwareStop::ConstPtr& msg){
	{
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		is_paused = msg->stop;
		controller->setSpeed(0, 0);
	}
}

void updateFeedback(const ros::TimerEvent& e){
	double leftCurrent, rightCurrent;
	double leftTemp, rightTemp;
	{
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		controller->getCurrent(leftCurrent, rightCurrent);
		controller->getTemp(leftTemp, rightTemp);
	}
	roboteq_driver::RoboteqFeedback feedback;
	feedback.header.stamp = ros::Time::now();
	feedback.left.temp.header = feedback.header;
	feedback.right.temp.header = feedback.header;

	feedback.left.temp.temperature = leftTemp;
	feedback.right.temp.temperature = rightTemp;

	feedback.left.current = leftCurrent;
	feedback.right.current = rightCurrent;

	feedback_pub.publish(feedback);
}

#define define_and_get_param(type, var_name, param_name, default_value)	\
	type var_name(default_value);						\
	if(!ros::param::get(param_name, var_name))\
		ROS_WARN_STREAM("Parameter <"<<param_name<<"> not set. Using default value '"<<var_name<<"'")
	

int main(int argc, char **argv){
  ros::init(argc, argv, "roboteq_manager");
  
  ros::NodeHandle n;

  ROS_INFO("Initializing RoboteqDevice");


  define_and_get_param(double, rotations_per_meter, "~rotations_per_meter", 1);
  define_and_get_param(double, max_mps, "~max_mps", 1);
  define_and_get_param(int, ppr, "~ppr", 250);
  define_and_get_param(double, feedback_rate, "~feedback_rate", 1);
  define_and_get_param(std::string, port, "~port", "/dev/ttyUSB0");

  controller = new roboteq_driver::RoboteqMotorController(rotations_per_meter, rotations_per_meter,
							  max_mps, max_mps,
							  ppr, ppr);

  controller->open(port);

  ROS_INFO("Initialization Complete");

  twist_sub = n.subscribe("cmd_vel", 1000, twistCallback);
  pause_sub = n.subscribe("/pause", 1000, pauseCallback);
  feedback_pub = n.advertise<roboteq_driver::RoboteqFeedback>("motor_feedback", 1000);
  ros::Timer feedback_timer = n.createTimer(ros::Duration(1/feedback_rate), updateFeedback);
  
  ros::spin();

  controller->setSpeed(0, 0);

  delete controller;

  return 0;
}
