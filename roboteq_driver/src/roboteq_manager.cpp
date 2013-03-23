#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roboteq_driver/roboteq_motor_controller.h"
#include "roboteq_driver/RoboteqGroupInfo.h"
#include "roboteq_driver/RoboteqGroupMotorControl.h"
#include "aero_srr_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

static boost::mutex controller_mutex;
roboteq_driver::RoboteqMotorController* controller;
ros::Publisher feedback_pub;
ros::Subscriber twist_sub;
ros::Subscriber pause_sub;
bool is_paused = false;


/*void drive(double u, double w){
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

void twistCallback(const roboteq_driver::RoboteqGroupMotorControl::ConstPtr& msg){
	drive(msg->linear.x, msg->angular.z);
	}*/
void pauseCallback(const aero_srr_msgs::SoftwareStop::ConstPtr& msg){
	{
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		is_paused = msg->stop;
		controller->setSpeed(0, 0);
	}
}

void updateFeedback(const ros::TimerEvent& e){
	double chan1Current, chan2Current;
	double chan1Temp, chan2Temp;
	uint64_t chan1Encoder, chan2Encoder;
	{
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		controller->getCurrent(leftCurrent, rightCurrent);
		controller->getTemp(leftTemp, rightTemp);
	}
	roboteq_driver::RoboteqGroupInfo feedback;
	feedback.header.stamp = ros::Time::now();

	roboteq_driver::RoboteqMotorInfo chan1Feedback;
	roboteq_driver::RoboteqMotorInfo chan2Feedback;
	chan1Feedback.temp.header = feedback.header;
	chan2Feedback.temp.header = feedback.header;

	chan1Feedback.temp.temperature = chan1Temp;
	chan2Feedback.temp.temperature = chan2Temp;

	chan1Feedback.current = chan1Current;
	chan2Feedback.current = chan2Current;

	chan1Feedback.encoder = chan1Encoder;
	chan2Feedback.encoder = chan2Encoder;

	feedback.motors[0] = chan1Feedback;
	feedback.motors[1] = chan2Feedback;

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
  feedback_pub = n.advertise<roboteq_driver::RoboteqGroupInfo>("motor_feedback", 1000);
  ros::Timer feedback_timer = n.createTimer(ros::Duration(1/feedback_rate), updateFeedback);
  
  ros::spin();

  controller->setSpeed(0, 0);

  delete controller;

  return 0;
}
