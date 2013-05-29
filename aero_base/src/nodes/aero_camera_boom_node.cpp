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
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include "hd_driver/HDMotorInfo.h"
#include "hd_driver/SetPositionAction.h"
//************ LOCAL DEPENDANCIES ****************//
#include "aero_base/SetBoomPosition.h"
//***********    NAMESPACES     ****************//


static ros::Publisher pose_pub;
static ros::Publisher joint_pub;
static boost::shared_ptr<actionlib::SimpleActionClient<hd_driver::SetPositionAction> > hd_control_srv;
static ros::ServiceServer pose_control_srv;

double ticks_per_radian = 4172151.34019;
int32_t zero_tick_position = 0;
std::string target_frame("/camera_boom_rot");
double min_angle = -M_PI-0.1;
double max_angle = M_PI+0.1;

bool poseControlCallback(aero_base::SetBoomPosition::Request  &req,
		     aero_base::SetBoomPosition::Response &res){
  if(req.max_velocity < 0.001){
    ROS_ERROR("Got request to go to angle with a max velocity of basically 0 (%f)", req.max_velocity);
    return false;
  }
  if(req.angle<min_angle){
    ROS_WARN("Requested angle (%f) was less than min angle (%f). Limiting to min angle", req.angle, min_angle);
    req.angle = min_angle;
  }
  if(req.angle>max_angle){
    ROS_WARN("Requested angle (%f) was greater than max angle (%f). Limiting to max angle", req.angle, max_angle);
    req.angle = max_angle;
  }
  ROS_INFO("Boom going to angle %f at max velocity %f", req.angle, req.max_velocity);
  hd_driver::SetPositionGoal hd_req;
  hd_req.position = (int32_t)(-req.angle * ticks_per_radian + zero_tick_position);
  hd_req.max_velocity = (float)(req.max_velocity * ticks_per_radian);
  hd_control_srv->sendGoalAndWait(hd_req);
  ROS_INFO("Boom reached goal %f", req.angle);
  return true;
}


void hdFeedbackCallback(const hd_driver::HDMotorInfo::ConstPtr& msg) {
  double angle = -(msg->position-zero_tick_position)/ticks_per_radian;
  geometry_msgs::Pose pose_msg;
  pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
  pose_pub.publish(pose_msg);
  
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = msg->header.stamp;
  joint_state.name.resize(1);
  joint_state.position.resize(1);
  joint_state.name[0] ="boom_joint";
  joint_state.position[0] = angle;
  joint_pub.publish(joint_state);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aero_camera_boom_node");
  ros::NodeHandle nh;

  if(!ros::param::get("~ticks_per_radian", ticks_per_radian))
    ROS_WARN_STREAM("Parameter <~ticks_per_radian> not set. Using default value '"<<ticks_per_radian<<"'");

  if(!ros::param::get("~zero_tick_position", zero_tick_position))
    ROS_WARN_STREAM("Parameter <~zero_tick_position> not set. Using default value '"<<zero_tick_position<<"'");
 
 if(!ros::param::get("~target_frame", target_frame))
    ROS_WARN_STREAM("Parameter <~target_frame> not set. Using default value '"<<target_frame<<"'");

	    
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
  joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  hd_control_srv = boost::shared_ptr<actionlib::SimpleActionClient<hd_driver::SetPositionAction> >(new actionlib::SimpleActionClient<hd_driver::SetPositionAction>(hd_control_service, true));
  pose_control_srv = nh.advertiseService(pose_control_service, poseControlCallback);

  hd_control_srv->waitForServer();

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
