/*
 * SyncStage.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: srr
 */

#include <object_locator/vision_pipeline/SyncStage.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

PLUGINLIB_DECLARE_CLASS(object_locator, SyncStage, object_locator::SyncStage,
		nodelet::Nodelet)

namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;

void SyncStage::onInit()
{
	loadParams();
	registerTopics();
}

void SyncStage::loadParams()
{
	this->left_input_topic_  ="/stereo_top/left/image_raw";
	this->right_input_topic_ ="/stereo_top/right/image_raw";
	this->output_topic_="disparity_stage/stereo_pair";
	this->it_ = new image_transport::ImageTransport(this->getNodeHandle());
	gotLeft = false;
	gotRight = false;

}

void SyncStage::registerTopics()
{
	this->image_left_  = it_->subscribeCamera(this->left_input_topic_,2,&SyncStage::leftImageCb,this);
	this->image_right_ = it_->subscribeCamera(this->right_input_topic_,2,&SyncStage::rightImageCb,this);
	this->sync_image_pub_ = this->getNodeHandle().advertise<object_locator::SyncImageMsg>(this->output_topic_,2);
}

void SyncStage::leftImageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	left_image_ = *msg;
	left_info_  = *cam_info;
	gotLeft = true;
	gotImages();
}

void SyncStage::rightImageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	right_image_ = *msg;
	right_info_  = *cam_info;
	gotRight = true;
	gotImages();
}

void SyncStage::gotImages()
{
	if(gotLeft && gotRight)
	{
		object_locator::SyncImageMsgPtr msg(new object_locator::SyncImageMsg);
		generateSyncMsg(*msg);
		this->sync_image_pub_.publish(msg);
		gotLeft = false;
		gotRight = false;
	}
}

void SyncStage::generateSyncMsg(object_locator::SyncImageMsg& msg)
{
	msg.left_image  = left_image_;
	msg.left_info   = left_info_;
	msg.right_image = right_image_;
	msg.right_info  = right_info_;
}
