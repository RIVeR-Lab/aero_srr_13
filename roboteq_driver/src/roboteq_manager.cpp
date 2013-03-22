#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roboteq_driver/roboteq_motor_controller.h"
#include "roboteq_driver/RoboteqFeedback.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

static boost::mutex controller_mutex;
roboteq_driver::RoboteqMotorController* controller;
ros::Publisher feedback_pub;
ros::Subscriber twist_sub;

void drive(double u, double w){
	//TODO do inverse kinematics here...
	double left_speed = u-w;
	double right_speed = u+w;
	{
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		controller->setSpeed(left_speed, right_speed);
	}
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
	drive(msg->linear.x, msg->angular.z);
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

int main(int argc, char **argv){
  ros::init(argc, argv, "roboteq_manager");
  
  ros::NodeHandle n;

  ROS_INFO("Initializing RoboteqDevice");


  double rotations_per_meter;
  double maxMPS;
  int ppr;
  double feedbackRate;
  std::string port;
  ros::param::get("~rotations_per_meter", rotations_per_meter);
  ros::param::get("~maxMPS", maxMPS);
  ros::param::get("~ppr", ppr);
  ros::param::get("~feedbackRate", feedbackRate);
  ros::param::get("~port", port);

  controller = new roboteq_driver::RoboteqMotorController(rotations_per_meter, rotations_per_meter,
							  maxMPS, maxMPS,
							  ppr, ppr);

  controller->open(port);

  ROS_INFO("Initialization Complete");

  twist_sub = n.subscribe("cmd_vel", 1000, twistCallback);
  feedback_pub = n.advertise<roboteq_driver::RoboteqFeedback>("motor_feedback", 1000);
  ros::Timer feedback_timer = n.createTimer(ros::Duration(1/feedbackRate), updateFeedback);
  
  ros::spin();

  delete controller;

  return 0;
}
