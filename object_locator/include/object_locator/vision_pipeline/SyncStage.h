/*
 * SyncStage.h
 *
 *  Created on: Apr 11, 2013
 *      Author: srr
 */

#ifndef SYNCSTAGE_H_
#define SYNCSTAGE_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <object_locator/SyncImageMsg.h>
#include <object_locator/SyncImagesAndDisparity.h>
#include <object_locator/typedefinitions.h>
#include <image_transport/image_transport.h>
#include <stereo_msgs/DisparityImage.h>
#include <message_filters/subscriber.h>

namespace object_locator
{
	class SyncStage:public nodelet::Nodelet
	{
	public:
		void onInit();

	protected:
		void loadParams();
		void registerTopics();
		virtual void imageCb(const object_locator::SyncImageMsgConstPtr& msg);
//		virtual void leftImageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
//		virtual void rightImageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
		virtual void disparityImageCb(const stereo_msgs::DisparityImageConstPtr& msg);
		virtual void generateSyncMsg(object_locator::SyncImagesAndDisparity& msg);
		virtual void gotImages();
		std::string left_input_topic_, right_input_topic_,output_topic_, disparity_input_topic_;
		image_transport::ImageTransport* it_;
//		image_transport::CameraSubscriber image_left_;
//		image_transport::CameraSubscriber image_right_;
		ros::Subscriber raw_image_sub_;
		ros::Subscriber disparity_;
		ros::Publisher sync_image_pub_;
//		sensor_msgs::Image left_image_;
//		sensor_msgs::Image right_image_;
		object_locator::SyncImageMsg raw_images_;
		stereo_msgs::DisparityImage disparity_image_;
//		sensor_msgs::CameraInfo left_info_;
//		sensor_msgs::CameraInfo right_info_;
		bool gotLeft_;
//		bool gotRight_;
		bool gotDisparity_;
	};

}

#endif /* SYNCSTAGE_H_ */
