/*
 * ros2cv.h
 *
 *  Created on: Mar 7, 2013
 *      Author: ssr
 */

#ifndef ROS2CV_H_
#define ROS2CV_H_

#include <image_geometry/stereo_camera_model.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <object_locator/ros2cv.h>
#include <opencv2/opencv.hpp>

namespace object_locator
{

class ImageConverter
{

public:
	ImageConverter();
	virtual ~ImageConverter();
	void processImage(const sensor_msgs::Image& msg,  cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW);
	void imageCbLeft(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
	void imageCbRight(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
	void computeDisparity();
	void computeDisparityCb(const ros::TimerEvent& event);


private:
	ros::Timer disp_timer;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber image_left_;
	image_transport::CameraSubscriber image_right_;
	image_transport::Publisher image_pub_;

	sensor_msgs::Image left_image;
	sensor_msgs::Image right_image;
	sensor_msgs::CameraInfo left_info;
	sensor_msgs::CameraInfo right_info;

	cv_bridge::CvImagePtr mat_left;
	cv_bridge::CvImagePtr mat_right;
	image_geometry::StereoCameraModel stereo_model;
	char* WINDOWLeft;
	char* WINDOWRight;
	char* WINDOWDisparity;
	cv::Mat disparity;
	bool gotLeft;
	bool gotRight;
};


};



#endif /* ROS2CV_H_ */
