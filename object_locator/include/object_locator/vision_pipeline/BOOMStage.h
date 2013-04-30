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

namespace object_locator
{
	class BOOMStage:public nodelet::Nodelet
	{
	public:
		void onInit();
	protected:
		void loadParams();
		void registerTopics();
		virtual void boomImageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);
		/**
		 * @author Samir Zutshi
		 * @brief Median filters, normalizes, and bandpass filters grass in the image
		 * @param [in] msg
		 * @param [out] normImage is the output image where the normalized and filtered image is stored
		 */
		virtual void grassRemove(const sensor_msgs::Image& msg, Mat_t& normImage);
		/**
		 * @author Samir Zutshi
		 * @brief Takes a grayscale image and detects for blobs and outputs its center.
		 * @param [in] img of type Mat CV_8UC1
		 */
		virtual void blobIdentify(Mat_t& img);
		virtual void generateMsg();

		ros::Subscriber sync_image_sub_;
		image_transport::ImageTransport* it_;
		image_transport::CameraSubscriber image_left_;


		std::string input_topic_, output_topic_;
		Mat_t load_;
	};
}

#endif /* BOOMSTAGE_H_ */
