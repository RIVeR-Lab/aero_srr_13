/*
 * RosStereoBridge.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/RosStereoBridge.h"

using namespace std;
using namespace cv;

void RosStereoBridge::stereoCallback(
	const sensor_msgs::ImageConstPtr& left_image_msg,
	const sensor_msgs::ImageConstPtr& right_image_msg,
	const stereo_msgs::DisparityImageConstPtr& disp_image_msg,
	const sensor_msgs::CameraInfoConstPtr& left_camera_info_msg,
	const sensor_msgs::CameraInfoConstPtr& right_camera_info_msg)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	cv_bridge::CvImageConstPtr cv_ptr_left = cv_bridge::toCvShare(left_image_msg, sensor_msgs::image_encodings::MONO8);
	cv_bridge::CvImageConstPtr cv_ptr_right = cv_bridge::toCvShare(right_image_msg, sensor_msgs::image_encodings::MONO8);
	cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvShare(disp_image_msg->image, disp_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	*frame_ = cv_ptr_disp->image;
	*lframe_ = cv_ptr_left->image;
	*rframe_ = cv_ptr_right->image;

	frame_no_++;
	newframe_=true;

}

RosStereoBridge::RosStereoBridge(std::string basetopic):
    it_(nh_),
	lframe_sub_( nh_, basetopic.append("/left/camera"), 1 ),
	rframe_sub_( nh_, basetopic.append("/right/camera"), 1 ),
	frame_sub_( nh_, basetopic.append("/disparity"), 1 ),
	lframe_info_sub_( nh_, basetopic.append("/right/camera_info"), 1 ),
	rframe_info_sub_( nh_, basetopic.append("/left/camera_info"), 1 ),
	sync_( SyncPolicy( 100 ), lframe_sub_, rframe_sub_, frame_sub_, lframe_info_sub_, rframe_info_sub_)
{
	newframe_=false;
	frame_=new Mat();
	lframe_=new Mat();
	rframe_=new Mat();
	sync_.registerCallback( boost::bind( &RosStereoBridge::stereoCallback, this, _1, _2, _3, _4, _5 ) );
}
vector<Mat*> RosStereoBridge::getNextStereoImage()
{
	vector<Mat*> stereopair;
	stereopair.push_back(lframe_);
	stereopair.push_back(rframe_);
	return(stereopair);
}
void RosStereoBridge::showPair(string wname)
{
	imshow(wname.append("left").c_str(),*lframe_);
	imshow(wname.append("right").c_str(),*rframe_);
}
cv::Mat* RosStereoBridge::getNextFrame()
{
	return(frame_);
}

RosStereoBridge::~RosStereoBridge() {
	delete(frame_);
	delete(lframe_);
	delete(rframe_);
}
