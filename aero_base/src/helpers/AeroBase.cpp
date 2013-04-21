/**
 * @file   AeroBase.cpp
 *
 * @date   Mar 11, 2013
 * @author Adam Panzica
 * @brief  Implementation of AeroBase class
 */

//*********** SYSTEM DEPENDANCIES ****************//
//************ LOCAL DEPENDANCIES ****************//
#include <aero_base/AeroBase.h>
#include "AeroBaseParams.h"
//***********    NAMESPACES     ****************//


using namespace aero_base;

AeroBase::AeroBase(ros::NodeHandle& nh, ros::NodeHandle& p_nh):
		nh_(nh),
		p_nh_(p_nh)
{
	ROS_INFO("Setting Up Aero Base Node...");
	this->loadParams();
	this->buildTransforms();
	this->registerTimers();
	ROS_INFO("Aero Base Node Running!");
}

void AeroBase::loadParams()
{
	ROS_INFO("Loading Params...");
	this->robot_ = "robot";
	this->nh_.getParam(TF_BASE_NAME, this->robot_);

	this->imu_ = "imu";
	this->nh_.getParam(TF_IMU_NAME, this->imu_);

	this->boom_  = "boom";
	this->nh_.getParam(TF_BOOM_NAME, this->imu_);

	this->left_cam_ = "left_camera";
	this->nh_.getParam(TF_LCAM_NAME, this->left_cam_);

	this->right_cam_= "right_camera";
	this->nh_.getParam(TF_RCAM_NAME, this->right_cam_);

	this->lidar_    = "laser";
	this->nh_.getParam(TF_LIDAR_NAME, this->lidar_);

	this->arm_base_ = "arm_base";
	this->nh_.getParam(TF_ARM_NAME, this->arm_base_);

}

void AeroBase::registerTimers()
{
	ROS_INFO("Registering Timers...");
	this->update_timer_ = this->nh_.createTimer(ros::Duration(1.0/10.0), &AeroBase::updateCB, this);
}

void AeroBase::buildTransforms()
{
	ROS_INFO("Building Transforms...");
	//build the IMU transform
	this->to_imu_.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
	this->to_imu_.setRotation(tf::Quaternion(0,0,0,1));

	//build the arm base transform
	this->to_arm_base_.setOrigin(tf::Vector3(0.0,1.0,0.0));
	this->to_arm_base_.setRotation(tf::Quaternion(0,0,0,1));

	//build the boom base transform
	this->to_boom_base_.setOrigin(tf::Vector3(0.0,0.0,1.0));
	this->to_boom_base_.setRotation(tf::Quaternion(0,0,0,1));

	//build the lidar base transform
	this->to_lidar_.setOrigin(tf::Vector3(1.0,0.0,0.0));
	tf::Quaternion lidarQ;
	lidarQ.setRPY(3.1415962, 0, 0);
	lidarQ.normalize();
	this->to_lidar_.setRotation(lidarQ);

	//build the right camera transform
	this->to_right_lower_camera_.setOrigin(tf::Vector3(0.0,1.0,1.0));
	this->to_right_lower_camera_.setRotation(tf::Quaternion(0,0,0,1));

	//build the left camera transform
	this->to_left_lower_camera_.setOrigin(tf::Vector3(1.0,1.0,1.0));
	this->to_left_lower_camera_.setRotation(tf::Quaternion(0,0,0,1));
}

void AeroBase::updateCB(const ros::TimerEvent& event)
{
	this->to_imu_bc_.sendTransform(tf::StampedTransform(this->to_imu_, ros::Time::now(), this->robot_, this->imu_));
	this->to_arm_base_bc_.sendTransform(tf::StampedTransform(this->to_arm_base_, ros::Time::now(), this->robot_, this->arm_base_));
	this->to_boom_base_bc_.sendTransform(tf::StampedTransform(this->to_boom_base_, ros::Time::now(), this->robot_, this->boom_));
	this->to_lidar_bc_.sendTransform(tf::StampedTransform(this->to_lidar_, ros::Time::now(), this->robot_, this->lidar_));
	this->to_left_lower_camera_bc_.sendTransform(tf::StampedTransform(this->to_left_lower_camera_, ros::Time::now(), this->robot_, this->left_cam_));
	this->to_right_lower_camera_bc_.sendTransform(tf::StampedTransform(this->to_right_lower_camera_, ros::Time::now(), this->robot_, this->right_cam_));
}
