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
	this->left_input_topic_  ="/stereo_camera/stereo_sync";
//	this->right_input_topic_ ="/stereo_camera/right/image_raw";
	this->disparity_input_topic_ ="/stereo_camera/disparity";
	this->output_topic_="disparity_stage/disparity";
	this->it_ = new image_transport::ImageTransport(this->getNodeHandle());
	gotLeft_ = false;
//	gotRight_ = false;
	gotDisparity_ = false;

}

void SyncStage::registerTopics()
{
	this->raw_image_sub_  = this->getNodeHandle().subscribe(this->left_input_topic_,2,&SyncStage::imageCb,this);
//	this->image_right_ = it_->subscribeCamera(this->right_input_topic_,2,&SyncStage::rightImageCb,this);
	this->disparity_ = it_->subscribe(this->disparity_input_topic_,2,&SyncStage::disparityImageCb,this);
	this->sync_image_pub_ = this->getNodeHandle().advertise<object_locator::SyncImagesAndDisparity>(this->output_topic_,2);
}

void SyncStage::imageCb(const object_locator::SyncImageMsgConstPtr& msg)
{
	raw_images_ = *msg;
	gotLeft_ = true;
	gotImages();
}
//
//void SyncStage::rightImageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
//{
//	right_image_ = *msg;
//	right_info_  = *cam_info;
//	gotRight_ = true;
//	gotImages();
//}

void SyncStage::disparityImageCb(const sensor_msgs::ImageConstPtr& msg)
{
	disparity_image_ = *msg;
	gotDisparity_ = true;
	gotImages();
}

void SyncStage::gotImages()
{
	if(gotLeft_ && gotDisparity_)
	{
		object_locator::SyncImagesAndDisparityPtr msg(new object_locator::SyncImagesAndDisparity);
		generateSyncMsg(*msg);
		this->sync_image_pub_.publish(msg);
		gotLeft_ = false;
		gotDisparity_ = false;
	}
}

void SyncStage::generateSyncMsg(object_locator::SyncImagesAndDisparity& msg)
{
	msg.images  		   = raw_images_;
//	msg.images.left_image  = left_image_;
//	msg.images.left_info   = left_info_;
//	msg.images.right_image = right_image_;
//	msg.images.right_info  = right_info_;
	msg.disparity_image    = disparity_image_;
}
