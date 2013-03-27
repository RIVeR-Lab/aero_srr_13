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
#include <nav_msgs/Odometry.h>
#include "roboteq_driver/roboteq_manager_lib.h"
#include "roboteq_driver/RoboteqGroupInfo.h"
//************ LOCAL DEPENDANCIES ****************//
//***********    NAMESPACES     ****************//


static ros::Publisher odom_pub;
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
  ROS_WARN("Publishing %f, %f", left_speed, right_speed);
  motor_controller->setRPM(left_speed*rotations_per_meter*60, right_speed*rotations_per_meter*60);
}

/**
 * callback for twist messages
 */
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  drive_vel(msg->linear.x, msg->linear.z);
}


void roboteqFeedbackCallback(const roboteq_driver::RoboteqGroupInfo::ConstPtr& msg) {
  roboteq_driver::RoboteqMotorInfo left = msg->motors[0];
  roboteq_driver::RoboteqMotorInfo right = msg->motors[1];
  double u1 = left.velocity/rotations_per_meter/60;
  double u2 = right.velocity/rotations_per_meter/60;

  nav_msgs::Odometry odom_msg;
  odom_msg.pose.covariance.assign(-1);
  odom_msg.twist.covariance.assign(-1);
  odom_msg.twist.covariance[0] = 1;
  odom_msg.twist.covariance[35] = 1;
  odom_msg.twist.twist.linear.x = (u1 + u2)/2;
  odom_msg.twist.twist.angular.z = (u2 - u1)/base_width;
  odom_pub.publish(odom_msg);

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aero_drive_node");
	ros::NodeHandle nh;

	if(!ros::param::get("~rotations_per_meter", rotations_per_meter))
	  ROS_WARN_STREAM("Parameter <~rotations_per_meter> not set. Using default value '"<<rotations_per_meter<<"'");
	if(!ros::param::get("~base_width", base_width))
	  ROS_WARN_STREAM("Parameter <~base_width> not set. Using default value '"<<base_width<<"'");

	    
	std::string roboteq_manager_cmd_topic = "drive_cmd";
	if(!ros::param::get("~roboteq_manager_cmd_topic", roboteq_manager_cmd_topic))
	  ROS_WARN_STREAM("Parameter <~roboteq_manager_cmd_topic> not set. Using default value '"<<roboteq_manager_cmd_topic<<"'");
	std::string roboteq_manager_feedback_topic = "aero_driver_feedback";
	if(!ros::param::get("~roboteq_manager_feedback_topic", roboteq_manager_feedback_topic))
	  ROS_WARN_STREAM("Parameter <~roboteq_manager_feedback_topic> not set. Using default value '"<<roboteq_manager_feedback_topic<<"'");

	std::string twist_topic = "cmd_vel";
	if(!ros::param::get("~twist_topic", twist_topic))
	  ROS_WARN_STREAM("Parameter <~twist_topic> not set. Using default value '"<<twist_topic<<"'");
	std::string odom_topic = "odom";
	if(!ros::param::get("~odom_topic", twist_topic))
	  ROS_WARN_STREAM("Parameter <~odom_topic> not set. Using default value '"<<odom_topic<<"'");


	motor_controller = new roboteq_driver::RoboteqManagerClient(nh, roboteq_manager_cmd_topic);

	ros::Subscriber twist_sub = nh.subscribe(twist_topic, 1000, twistCallback);
	ros::Subscriber feedback_sub = nh.subscribe(roboteq_manager_feedback_topic, 1000, roboteqFeedbackCallback);
	odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);

	ros::spin();
}
