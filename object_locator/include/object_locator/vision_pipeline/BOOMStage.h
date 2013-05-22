/*
 * BOOMStage.h
 *
 *  Created on: Apr 19, 2013
 *      Author: srr
 */

#ifndef BOOMSTAGE_H_
#define BOOMSTAGE_H_


#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <object_locator/SyncImageMsg.h>
#include <object_locator/typedefinitions.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <object_locator/classifierTypes.h>
#include <object_locator/DetectionManager.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/foreach.hpp>

namespace object_locator
{
	class BOOMStage:public nodelet::Nodelet
	{
	public:
		void onInit();
	protected:
		void loadParams();
		void registerTopics();
		virtual void boomImageCbleft(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);
		virtual void boomImageCbright(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);
		virtual void computeDisparityCb();
		/**
		 * @author Samir Zutshi
		 * @brief Median filters, normalizes, and bandpass filters grass in the image
		 * @param [in] msg
		 * @param [out] normImage is the output image where the normalized and filtered image is stored
		 */
		virtual void grassRemove(const sensor_msgs::Image& msg, Mat_t& normImage);
		virtual void maskCreate(const sensor_msgs::Image& msg, Mat_t& maskt);
		/**
		 * @author Samir Zutshi
		 * @brief Takes a grayscale image and detects for blobs and outputs its center.
		 * @param [in] img of type Mat CV_8UC1
		 */
		virtual void blobIdentify(const Mat_t& img, Mat_t& mask, Mat_t& final);
		virtual void detectAnomalies(Mat_t& img, Mat_t& mask);
		virtual void showAlpha(Mat_t& src, Mat_t& fMask);
		virtual void computeDisparity();

		virtual void generateMsg();

		ros::Subscriber sync_image_sub_;
		image_transport::ImageTransport* it_;
		image_transport::CameraSubscriber image_left_, image_right_;
		sensor_msgs::CameraInfo left_info_,right_info_;
		Mat_t left_image_,right_image_;
		bool got_left_,got_right_;
		int HORIZON_TOP_;
		int HORIZON_BTM_;
		cv::Vec3b ZeroV,White;

		ros::Publisher pose_array_pub_;

		typedef std::pair<int, int> PixPoint_t;
		typedef std::pair<PixPoint_t, object_type> Detection_t;
		typedef boost::shared_ptr<Detection_t> DetectionPtr_t;
		std::vector<DetectionPtr_t> detection_list_;
		tf::TransformListener optimus_prime;

		//***sherlock Parameters*****/
		double thresh_dist_, growth_rate_, shrink_rate_, thresh_det_;
		object_locator::DetectionManager* watson_;

		image_geometry::StereoCameraModel stereo_model;

		std::string left_input_topic_, right_input_topic_, output_topic_;
		Mat_t load_;
	};
}

#endif /* BOOMSTAGE_H_ */
