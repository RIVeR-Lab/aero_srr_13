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
#include <tf/transform_datatypes.h>
#include "hd_driver/HDMotorInfo.h"
#include "hd_driver/SetPosition.h"
//************ LOCAL DEPENDANCIES ****************//
#include "aero_base/SetPose.h"
//***********    NAMESPACES     ****************//


static ros::Publisher pose_pub;
static ros::ServiceClient hd_control_srv;
static ros::ServiceServer pose_control_srv;

double ticks_per_radian = 1.0;

bool poseControlCallback(aero_base::SetPose::Request  &req,
		     aero_base::SetPose::Response &res){
  hd_driver::SetPosition::Request hd_req;
  hd_req.position = (int32_t)(tf::getYaw(req.pose.orientation) * ticks_per_radian);
  hd_driver::SetPosition::Response hd_res;
  return hd_control_srv.call(hd_req, hd_res);
}


void hdFeedbackCallback(const hd_driver::HDMotorInfo::ConstPtr& msg) {
  geometry_msgs::Pose pose_msg;
  pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, msg->position/ticks_per_radian);
  pose_pub.publish(pose_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aero_camera_boom_node");
  ros::NodeHandle nh;

  if(!ros::param::get("~ticks_per_radian", ticks_per_radian))
    ROS_WARN_STREAM("Parameter <~ticks_per_radian> not set. Using default value '"<<ticks_per_radian<<"'");

	    
  std::string hd_control_service = "hd_control";
  if(!ros::param::get("~hd_control_service", hd_control_service))
    ROS_WARN_STREAM("Parameter <~hd_control_service> not set. Using default value '"<<hd_control_service<<"'");

  std::string pose_control_service = "hd_control";
  if(!ros::param::get("~pose_control_service", pose_control_service))
    ROS_WARN_STREAM("Parameter <~pose_control_service> not set. Using default value '"<<pose_control_service<<"'");


  std::string hd_feedback_topic = "hd_info";
  if(!ros::param::get("~hd_feedback_topic", hd_feedback_topic))
    ROS_WARN_STREAM("Parameter <~hd_feedback_topic> not set. Using default value '"<<hd_feedback_topic<<"'");

  std::string pose_topic = "camera_pose";
  if(!ros::param::get("~pose_topic", pose_topic))
    ROS_WARN_STREAM("Parameter <~pose_topic> not set. Using default value '"<<pose_topic<<"'");


  ros::Subscriber feedback_sub = nh.subscribe(hd_feedback_topic, 1000, hdFeedbackCallback);
  pose_pub = nh.advertise<geometry_msgs::Pose>(pose_topic, 1000);

  hd_control_srv = nh.serviceClient<hd_driver::SetPosition>(hd_control_service);
  pose_control_srv = nh.advertiseService(pose_control_service, poseControlCallback);


  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
