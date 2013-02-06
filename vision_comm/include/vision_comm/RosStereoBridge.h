/*
 * RosStereoBridge.h
 *
 *  Created on: Jan 30, 2013
 *      Author: bpwiselybabu
 */

#ifndef ROSSTEREOBRIDGE_H_
#define ROSSTEREOBRIDGE_H_

#include "IOImages.h"

#include <ros/ros.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <vector>

typedef message_filters::sync_policies::ApproximateTime<
		sensor_msgs::Image, sensor_msgs::Image, stereo_msgs::DisparityImage,
		sensor_msgs::CameraInfo, sensor_msgs::CameraInfo
		> SyncPolicy;

class RosStereoBridge: public IOImages {
	/*
	 * the frame_ will be used for the depth image
	 * the lframe and the rframe will be stored here.
	 */
	cv::Mat* lframe_;
	cv::Mat* rframe_;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	message_filters::Subscriber< sensor_msgs::Image> lframe_sub_;
	message_filters::Subscriber< sensor_msgs::Image> rframe_sub_;
	message_filters::Subscriber< stereo_msgs::DisparityImage> frame_sub_;
	message_filters::Subscriber< sensor_msgs::CameraInfo> lframe_info_sub_;
	message_filters::Subscriber< sensor_msgs::CameraInfo> rframe_info_sub_;
	message_filters::Synchronizer< SyncPolicy > sync_;

	bool newframe_;

	void stereoCallback(
		const sensor_msgs::ImageConstPtr& left_image_msg,
		const sensor_msgs::ImageConstPtr& right_image_msg,
		const stereo_msgs::DisparityImageConstPtr& disp_image_msg,
		const sensor_msgs::CameraInfoConstPtr& left_camera_info_msg,
		const sensor_msgs::CameraInfoConstPtr& right_camera_info_msg);

public:
	RosStereoBridge(std::string basetopic);
	virtual ~RosStereoBridge();
	std::vector<cv::Mat *> getNextStereoImage();
	void showPair(std::string wname);
	cv::Mat* getNextFrame();

};

#endif /* ROSSTEREOBRIDGE_H_ */
