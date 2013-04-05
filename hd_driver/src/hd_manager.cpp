#include "ros/ros.h"
#include "hd_driver/hd_motor_controller.h"
#include "hd_driver/HDMotorInfo.h"
#include "hd_driver/SetPosition.h"
#include "aero_srr_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>

static boost::mutex controller_mutex;
hd_driver::HDMotorController* controller;
ros::Publisher feedback_pub;
ros::ServiceServer control_srv;
ros::Subscriber pause_sub;

bool controlCallback(hd_driver::SetPosition::Request  &req,
		     hd_driver::SetPosition::Response &res){
  ROS_INFO("Setting position to: [%d]", req.position);
  {
    boost::lock_guard<boost::mutex> lock(controller_mutex);
    controller->set_position(req.position);
  }
  while(1){
    {
      boost::lock_guard<boost::mutex> lock(controller_mutex);
      if(!(controller->get_status()&STATUS_TRAJECTORY_RUNNING))
	break;
    }
    usleep(100000);
  }
  {
    boost::lock_guard<boost::mutex> lock(controller_mutex);
    if(controller->get_trajectory_status()&TRAJECTORY_MOVE_ABORTED)
      return false;
    return true;
  }
}


void pauseCallback(const aero_srr_msgs::SoftwareStop::ConstPtr& msg){
  //TODO implement pause
}

void feedbackTimerCallback(const ros::TimerEvent& e){
  hd_driver::HDMotorInfo msg;
  {
    boost::lock_guard<boost::mutex> lock(controller_mutex);
    msg.position = controller->get_position();
  }
  feedback_pub.publish(msg);
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
  define_and_get_param(std::string, port, "~port", "/dev/ttyUSB0");
  define_and_get_param(std::string, control_service, "~control_service", "hd_control");
  define_and_get_param(std::string, info_topic, "~info_topic", "hd_info");
  define_and_get_param(std::string, pause_topic, "~pause_topic", "/pause");

  controller = new hd_driver::HDMotorController();

  controller->open(port);

  ROS_INFO("Initialization Complete");

  control_srv = n.advertiseService(control_service, controlCallback);
  pause_sub = n.subscribe(pause_topic, 1000, pauseCallback);
  feedback_pub = n.advertise<hd_driver::HDMotorInfo>(info_topic, 1000);
  ros::Timer feedback_timer = n.createTimer(ros::Duration(1/feedback_rate), feedbackTimerCallback);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  delete controller;

  return 0;
}
