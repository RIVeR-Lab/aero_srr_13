#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roboteq_driver/roboteq_motor_controller.h"
#include "roboteq_driver/RoboteqGroupInfo.h"
#include "roboteq_driver/RoboteqGroupMotorControl.h"
#include "aero_srr_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>

static boost::mutex controller_mutex;
roboteq_driver::RoboteqMotorController* controller;
ros::Publisher feedback_pub;
ros::Subscriber twist_sub;
ros::Subscriber pause_sub;
bool is_paused = false;


void controlCallback(const roboteq_driver::RoboteqGroupMotorControl::ConstPtr& msg){
  {
    boost::lock_guard<boost::mutex> lock(controller_mutex);
    BOOST_FOREACH(roboteq_driver::RoboteqMotorControl motorControl, msg->motors){
      if(is_paused)
	controller->setRPM(motorControl.channel, 0);
      else if(motorControl.control_mode==roboteq_driver::RoboteqMotorControl::RPM)
	controller->setRPM(motorControl.channel, motorControl.setpoint);
      else
	ROS_WARN("Control mode %d not implemented", motorControl.control_mode);
    }
  }
}
void pauseCallback(const aero_srr_msgs::SoftwareStop::ConstPtr& msg){
	{
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		is_paused = msg->stop;
		controller->setRPM(1, 0);
		controller->setRPM(2, 0);
	}
}

static void getFeedback(uint8_t chan, roboteq_driver::RoboteqMotorInfo& chanFeedback){
	chanFeedback.channel = chan;

	controller->getCurrent(chan, chanFeedback.current);

	controller->getTemp(chan, chanFeedback.temp.temperature);
	chanFeedback.temp.header.stamp = ros::Time::now();

	controller->getPosition(chan, chanFeedback.position);
}
void updateFeedback(const ros::TimerEvent& e){

	roboteq_driver::RoboteqGroupInfo feedback;
	roboteq_driver::RoboteqMotorInfo chan1Feedback;
	roboteq_driver::RoboteqMotorInfo chan2Feedback;
	{

		boost::lock_guard<boost::mutex> lock(controller_mutex);
		feedback.header.stamp = ros::Time::now();

		getFeedback(1, chan1Feedback);
		getFeedback(2, chan2Feedback);
	}

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

  define_and_get_param(double, max_rpm, "~max_rpm", 250);
  define_and_get_param(int, ppr, "~ppr", 250);
  define_and_get_param(double, feedback_rate, "~feedback_rate", 1);
  define_and_get_param(std::string, port, "~port", "/dev/ttyUSB0");

  controller = new roboteq_driver::RoboteqMotorController(max_rpm, max_rpm, ppr, ppr);

  controller->open(port);

  ROS_INFO("Initialization Complete");

  //twist_sub = n.subscribe("cmd_vel", 1000, twistCallback);
  pause_sub = n.subscribe("/pause", 1000, pauseCallback);
  feedback_pub = n.advertise<roboteq_driver::RoboteqGroupInfo>("motor_feedback", 1000);
  ros::Timer feedback_timer = n.createTimer(ros::Duration(1/feedback_rate), updateFeedback);
  
  ros::spin();

  controller->setRPM(0, 0);

  delete controller;

  return 0;
}
