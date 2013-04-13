#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include "roboteq_driver/roboteq_manager_lib.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"


/**
 * a stupid simple test program which responds to twist or joy messages
 */

roboteq_driver::RoboteqManagerClient* client;

void drive(double u, double w){
  client->setPower(-u+w, u+w);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
  drive(msg->linear.x, msg->linear.z);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  drive(msg->axes[1], msg->axes[0]);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "roboteq_manager_twist_client");
  
  ros::NodeHandle n;

  client = new roboteq_driver::RoboteqManagerClient(n, "roboteq_control");

  ros::Subscriber twist_sub = n.subscribe("cmd_vel", 1000, twistCallback);

  ros::Subscriber joy_sub = n.subscribe("joy", 1000, joyCallback);

  ros::spin();
  
  delete client;
  
  return 0;
}
