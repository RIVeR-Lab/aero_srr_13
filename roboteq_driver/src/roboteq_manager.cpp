#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roboteq_driver/roboteq_motor_controller.h"
#include "roboteq_driver/RoboteqGroupInfo.h"
#include "roboteq_driver/RoboteqGroupMotorControl.h"
#include "aero_srr_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>
#include "device_driver_base/driver_util.h"

static boost::mutex controller_mutex;
roboteq_driver::RoboteqMotorController* controller;
ros::Publisher feedback_pub;
ros::Subscriber control_sub;
ros::Subscriber pause_sub;

roboteq_driver::RoboteqGroupMotorControl::ConstPtr control_msg;
bool is_new = false;
bool is_paused = false;

ros::Timer feedback_timer;
ros::Timer control_timer;

void controlTimerCallback(const ros::TimerEvent& e){
  try{
  roboteq_driver::RoboteqGroupMotorControl::ConstPtr& msg = control_msg;
  if(!is_new || !msg)
    return;
  {
    boost::lock_guard<boost::mutex> lock(controller_mutex);
    is_new = false;
    BOOST_FOREACH(roboteq_driver::RoboteqMotorControl motorControl, msg->motors){
      if(is_paused)
	controller->setPower(motorControl.channel, 0);
      else if(motorControl.control_mode==roboteq_driver::RoboteqMotorControl::RPM)
	controller->setRPM(motorControl.channel, motorControl.setpoint);
      else if(motorControl.control_mode==roboteq_driver::RoboteqMotorControl::POWER)
	controller->setPower(motorControl.channel, motorControl.setpoint);
      else
	ROS_WARN("Control mode %d not implemented", motorControl.control_mode);
    }
  }
  } catch(...){
    feedback_timer.stop();
    control_timer.stop();
    throw;
  }
}
void controlCallback(const roboteq_driver::RoboteqGroupMotorControl::ConstPtr& msg){
  control_msg = msg;
  is_new = true;
}
void pauseCallback(const aero_srr_msgs::SoftwareStop::ConstPtr& msg){
  boost::lock_guard<boost::mutex> lock(controller_mutex);
  is_paused = msg->stop;
  controller->setPower(1, 0);
  controller->setPower(2, 0);
}

static void getFeedback(uint8_t chan, roboteq_driver::RoboteqMotorInfo& chanFeedback){
  chanFeedback.channel = chan;

  controller->getCurrent(chan, chanFeedback.current);

  controller->getTemp(chan, chanFeedback.temp.temperature);
  chanFeedback.temp.header.stamp = ros::Time::now();

  controller->getPosition(chan, chanFeedback.position);

  controller->getVelocity(chan, chanFeedback.velocity);
}
void feedbackTimerCallback(const ros::TimerEvent& e){
  try{
    roboteq_driver::RoboteqGroupInfo feedback;
    roboteq_driver::RoboteqMotorInfo chan1Feedback;
    roboteq_driver::RoboteqMotorInfo chan2Feedback;
    {
      boost::lock_guard<boost::mutex> lock(controller_mutex);
      feedback.header.stamp = ros::Time::now();

      getFeedback(1, chan1Feedback);
      getFeedback(2, chan2Feedback);
    }

    feedback.motors.push_back(chan1Feedback);
    feedback.motors.push_back(chan2Feedback);

    feedback_pub.publish(feedback);
  } catch(...){
    feedback_timer.stop();
    control_timer.stop();
    throw;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "roboteq_manager");
  
  ros::NodeHandle n;

  ROS_INFO("Initializing Roboteq Device");

  define_and_get_param(double, max_rpm, "~max_rpm", 250);
  define_and_get_param(int, ppr, "~ppr", 250);
  define_and_get_param(double, feedback_rate, "~feedback_rate", 1);
  define_and_get_param(double, control_rate, "~control_rate", 10);
  define_and_get_param(std::string, port, "~port", "/dev/ttyACM0");
  define_and_get_param(std::string, control_topic, "~control_topic", "roboteq_control");
  define_and_get_param(std::string, info_topic, "~info_topic", "roboteq_info");
  define_and_get_param(std::string, pause_topic, "~pause_topic", "/pause");

  controller = new roboteq_driver::RoboteqMotorController(max_rpm, max_rpm, ppr, ppr);


  while(ros::ok()){
    try{
      controller->open(port);
    } catch(device_driver::Exception& e){
      ROS_ERROR_STREAM("Error opening port: "<<e.what());
      ros::Duration(1).sleep();
      continue;
    }

    try{
      ROS_INFO("Roboteq Device Initialization Complete");

      control_sub = n.subscribe(control_topic, 1000, controlCallback);
      pause_sub = n.subscribe(pause_topic, 1000, pauseCallback);
      feedback_pub = n.advertise<roboteq_driver::RoboteqGroupInfo>(info_topic, 1000);
      feedback_timer = n.createTimer(ros::Duration(1/feedback_rate), feedbackTimerCallback);
      control_timer = n.createTimer(ros::Duration(1/control_rate), controlTimerCallback);
  
      ros::spin();
      controller->setPower(0, 0);
    } catch(device_driver::Exception& e){
      feedback_timer.stop();
      control_timer.stop();
      controller->close();
      ROS_ERROR_STREAM("An error occured while communicating with device: "<<e.what());
      ros::Duration(0.5).sleep();
    }
  }


  delete controller;

  return 0;
}
