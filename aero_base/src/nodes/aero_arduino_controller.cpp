/**
 * @file   aero_arduino_controller.cpp
 *
 * @date   Mar 26, 2013
 * @author Mitchell Wills
 * @brief  Implementation for the aero_drive_node ROS node
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <aero_srr_msgs/AeroState.h>
#include <robot_base_msgs/SoftwareStop.h>
//************ LOCAL DEPENDANCIES ****************//
//***********    NAMESPACES     ****************//

std::string pause_topic("/pause");
std::string arduino_pause_topic("/arduino_pause");

std::string state_topic("/state");
std::string status_rate_topic("/arduino_rate");

static ros::Publisher pause_pub;
static ros::Subscriber arduino_pause_sub;

static ros::Subscriber state_sub;
static ros::Publisher status_rate_pub;


bool last_pause_state = true;
void pauseCallback(const std_msgs::BoolConstPtr& is_paused){
  ROS_DEBUG_STREAM("Reveived software stop message from arduino: "<<is_paused->data);
  if(is_paused->data!=last_pause_state){
    robot_base_msgs::SoftwareStop msg;
    msg.stop = is_paused->data;
    msg.header.stamp = ros::Time::now();
    if(msg.stop)
      msg.message = "Hardware pause was pressed";
    else
      msg.message = "Hardware pause was released";
    last_pause_state = msg.stop;
    ROS_INFO_STREAM(msg.message);
    pause_pub.publish(msg);
  }
}

void publish_status_rate(double rate){
  std_msgs::Float32 msg;
  msg.data = rate;
  status_rate_pub.publish(msg);
}
void stateCallback(const aero_srr_msgs::AeroStateConstPtr& state_msg){
  switch(state_msg->state){
  case aero_srr_msgs::AeroState::ERROR:
    publish_status_rate(20);
    break;
  case aero_srr_msgs::AeroState::SEARCH:
    publish_status_rate(1);
    break;
  case aero_srr_msgs::AeroState::NAVOBJ:
    publish_status_rate(1);
    break;
  case aero_srr_msgs::AeroState::COLLECT:
    publish_status_rate(1);
    break;
  case aero_srr_msgs::AeroState::PICKUP:
    publish_status_rate(1);
    break;
  case aero_srr_msgs::AeroState::HOME:
    publish_status_rate(1);
    break;
  case aero_srr_msgs::AeroState::PAUSE:
    publish_status_rate(0.0);
    break;
  case aero_srr_msgs::AeroState::MANUAL:
    publish_status_rate(0.25);
    break;
  default:
    publish_status_rate(10);
  }
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "aero_arduino_controller");
	ros::NodeHandle nh;

	if(!ros::param::get("~pause_topic", pause_topic))
	  ROS_WARN_STREAM("Parameter <~pause_topic> not set. Using default value '"<<pause_topic<<"'");
	if(!ros::param::get("~arduino_pause_topic", arduino_pause_topic))
	  ROS_WARN_STREAM("Parameter <~arduino_pause_topic> not set. Using default value '"<<arduino_pause_topic<<"'");

	if(!ros::param::get("~state_topic", state_topic))
	  ROS_WARN_STREAM("Parameter <~status_topic> not set. Using default value '"<<state_topic<<"'");
	if(!ros::param::get("~status_rate_topic", status_rate_topic))
	  ROS_WARN_STREAM("Parameter <~status_rate_topic> not set. Using default value '"<<status_rate_topic<<"'");

	arduino_pause_sub = nh.subscribe(arduino_pause_topic, 1000, pauseCallback);
	pause_pub = nh.advertise<robot_base_msgs::SoftwareStop>(pause_topic, 1000, true);

	state_sub = nh.subscribe(state_topic, 1000, stateCallback);
	status_rate_pub = nh.advertise<std_msgs::Float32>(status_rate_topic, 1000, true);

	ros::spin();
}
