/**
 * @file   aero_drive_node.cpp
 *
 * @date   Mar 26, 2013
 * @author Mitchell Wills
 * @brief  Implementation for the aero_drive_node ROS node
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "roboteq_driver/roboteq_manager_lib.h"
//************ LOCAL DEPENDANCIES ****************//
//***********    NAMESPACES     ****************//



static roboteq_driver::RoboteqManagerClient* motor_controller;
double rotations_per_meter = 1.0;
double base_width = 0.6;

/**
 * The function that actually commands the robot to drive
 * @param u the forward velocity of the robot
 * @param w the angular velocity of the robot
 */
void drive_vel(double u, double w){
  double left_speed = u - (base_width/2 * w /2);
  double right_speed = u + (base_width/2 * w /2);

  motor_controller->setRPM(left_speed*rotations_per_meter*60, right_speed*rotations_per_meter*60);
}

/**
 * callback for twist messages
 */
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  drive_vel(msg->linear.x, msg->linear.z);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aero_drive_node");
	ros::NodeHandle nh;

	if(!ros::param::get("~rotations_per_meter", rotations_per_meter))
	  ROS_WARN_STREAM("Parameter <~rotations_per_meter> not set. Using default value '"<<rotations_per_meter<<"'");
	if(!ros::param::get("~base_width", base_width))
	  ROS_WARN_STREAM("Parameter <~base_width> not set. Using default value '"<<base_width<<"'");

	    
	std::string roboteq_manager_topic = "drive_cmd";
	if(!ros::param::get("~roboteq_manager_topic", roboteq_manager_topic))
	  ROS_WARN_STREAM("Parameter <~roboteq_manager_topic> not set. Using default value '"<<roboteq_manager_topic<<"'");
	std::string twist_topic = "cmd_vel";
	if(!ros::param::get("~twist_topic", twist_topic))
	  ROS_WARN_STREAM("Parameter <~twist_topic> not set. Using default value '"<<twist_topic<<"'");


	motor_controller = new roboteq_driver::RoboteqManagerClient(nh, roboteq_manager_topic);

	ros::Subscriber twist_sub = nh.subscribe(twist_topic, 1000, twistCallback);

	ros::spin();
}
