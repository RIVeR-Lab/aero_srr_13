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
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include "roboteq_driver/roboteq_manager_lib.h"
#include "roboteq_driver/RoboteqGroupInfo.h"
#include <tf/tf.h>
//************ LOCAL DEPENDANCIES ****************//
//***********    NAMESPACES     ****************//


static ros::Publisher odom_pub;
static ros::Publisher twist_pub;
static ros::Publisher joint_pub;
static roboteq_driver::RoboteqManagerClient* motor_controller;
double driver_rotations_per_rotation = 1.0;
double rotations_per_meter = 1.0;
double base_width = 0.554;
double base_length = 0.52;
std::string pose_frame("/odom");
std::string twist_frame("/base_footprint");

/**
 * The function that actually commands the robot to drive
 * @param u the forward velocity of the robot
 * @param w the angular velocity of the robot
 */
void drive_vel(double u, double w){
  double r = sqrt(base_length*base_length/4 + base_width*base_width/4);//distance from turning center to wheel
  double u1 = u - 2 * r*r / base_width * w;
  double u2 = u + 2 * r*r / base_width * w;
  motor_controller->setRPM(-u1*rotations_per_meter*driver_rotations_per_rotation*60, u2*rotations_per_meter*driver_rotations_per_rotation*60);
}

/**
 * callback for twist messages
 */
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  drive_vel(msg->linear.x, msg->angular.z);
}


double x = 0;
double y = 0;
double t = 0;//theta
double last_u = 0;
double last_w = 0;
ros::Time last_time;

void integrateVelocity(double u, double w, const ros::Time& time){
  double dt = (time-last_time).toSec();
  double dx, dy;
  
  if(w!=0){
    double a = w*dt;//angle rotated
    double r = u/w;//radius of arc driven
  
    dx = r*sin(t+a) - r*sin(t);
    dy = -r*cos(t+a) + r*cos(t);
  }
  else{//w==0
    double d = u*dt;
    dx = d*cos(t);
    dy = d*sin(t);
  }

  x += dx;
  y += dy;

  t += last_w*dt;
  last_time = time;
  last_u = u;
  last_w = w;
}

void roboteqFeedbackCallback(const roboteq_driver::RoboteqGroupInfo::ConstPtr& msg) {
  roboteq_driver::RoboteqMotorInfo left = msg->motors[0];
  roboteq_driver::RoboteqMotorInfo right = msg->motors[1];
  //positions in rad
  double t1 = -left.position/driver_rotations_per_rotation*M_PI*2;
  double t2 = right.position/driver_rotations_per_rotation*M_PI*2;
  //angular velocity in rad/s
  double w1 = -left.velocity/driver_rotations_per_rotation/60*M_PI*2;
  double w2 = right.velocity/driver_rotations_per_rotation/60*M_PI*2;
  //linear velocity
  double u1 = -left.velocity/driver_rotations_per_rotation/rotations_per_meter/60;
  double u2 = right.velocity/driver_rotations_per_rotation/rotations_per_meter/60;

  double r = sqrt(base_length*base_length/4 + base_width*base_width/4);//distance from turning center to wheel
  double u = (u1 + u2)/2;
  double w = (u2 - u1)*base_width/(4*r*r);
  integrateVelocity(u, w, msg->header.stamp);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = pose_frame;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.pose.covariance.assign(0);
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(t);

  odom_msg.pose.covariance[0] = 0.1;
  odom_msg.pose.covariance[7] = 0.1;
  odom_msg.pose.covariance[14] = 99999;
  odom_msg.pose.covariance[21] = 99999;
  odom_msg.pose.covariance[28] = 99999;
  odom_msg.pose.covariance[35] = 0.1;

  odom_msg.child_frame_id = twist_frame;
  odom_msg.twist.covariance.assign(0);
  odom_msg.twist.covariance[0] = 0.1;
  odom_msg.pose.covariance[7] = 99999;
  odom_msg.pose.covariance[14] = 99999;
  odom_msg.pose.covariance[21] = 99999;
  odom_msg.pose.covariance[28] = 99999;
  odom_msg.twist.covariance[35] = 0.01;
  odom_msg.twist.twist.linear.x = u;
  odom_msg.twist.twist.angular.z = w;
  odom_pub.publish(odom_msg);


  geometry_msgs::TwistWithCovarianceStamped twist_msg;
  twist_msg.twist = odom_msg.twist;
  twist_msg.header.frame_id = twist_frame;
  twist_msg.header.stamp = msg->header.stamp;
  twist_pub.publish(twist_msg);


  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(4);
  joint_state.position.resize(4);
  joint_state.velocity.resize(4);

  joint_state.name[0] ="joint_front_left_wheel";
  joint_state.position[0] = t1;
  joint_state.velocity[0] = w1;
  joint_state.name[1] ="joint_back_left_wheel";
  joint_state.position[1] = t1;
  joint_state.velocity[1] = w1;

  joint_state.name[2] ="joint_front_right_wheel";
  joint_state.position[2] = t2;
  joint_state.velocity[2] = w2;
  joint_state.name[3] ="joint_back_right_wheel";
  joint_state.position[3] = t2;
  joint_state.velocity[3] = w2;

  joint_pub.publish(joint_state);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aero_drive_node");
	ros::NodeHandle nh;

	if(!ros::param::get("~driver_rotations_per_rotation", driver_rotations_per_rotation))
	  ROS_WARN_STREAM("Parameter <~driver_rotations_per_rotation> not set. Using default value '"<<driver_rotations_per_rotation<<"'");
	if(!ros::param::get("~rotations_per_meter", rotations_per_meter))
	  ROS_WARN_STREAM("Parameter <~rotations_per_meter> not set. Using default value '"<<rotations_per_meter<<"'");
	if(!ros::param::get("~base_width", base_width))
	  ROS_WARN_STREAM("Parameter <~base_width> not set. Using default value '"<<base_width<<"'");
	if(!ros::param::get("~base_length", base_length))
	  ROS_WARN_STREAM("Parameter <~base_length> not set. Using default value '"<<base_length<<"'");

	    
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
	if(!ros::param::get("~odom_topic", odom_topic))
	  ROS_WARN_STREAM("Parameter <~odom_topic> not set. Using default value '"<<odom_topic<<"'");

	if(!ros::param::get("~pose_frame", pose_frame))
	  ROS_WARN_STREAM("Parameter <~pose_frame> not set. Using default value '"<<pose_frame<<"'");
	if(!ros::param::get("~twist_frame", twist_frame))
	  ROS_WARN_STREAM("Parameter <~twist_frame> not set. Using default value '"<<twist_frame<<"'");


	motor_controller = new roboteq_driver::RoboteqManagerClient(nh, roboteq_manager_cmd_topic);

	ros::Subscriber twist_sub = nh.subscribe(twist_topic, 1000, twistCallback);
	ros::Subscriber feedback_sub = nh.subscribe(roboteq_manager_feedback_topic, 1000, roboteqFeedbackCallback);
	odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
	twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("drive_twist", 1000);
	joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

	ros::spin();
}
