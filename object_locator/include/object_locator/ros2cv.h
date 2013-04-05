/*
 * ros2cv.h
 *
 *  Created on: Mar 7, 2013
 *      Author: ssr
 */

#ifndef ROS2CV_H_
#define ROS2CV_H_


#include <object_locator/typedefinitions.h>
#include <image_geometry/stereo_camera_model.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <cxcore.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>
#include <tf/transform_listener.h>
#include <object_locator/DetectionManager.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>





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
	void detectAndDisplay( const sensor_msgs::Image& msg, cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW);
	void buildMsg(const tf::Point& point, geometry_msgs::PoseStamped& msg) const;
	void saveImage(const sensor_msgs::Image& msg,cv_bridge::CvImagePtr& cv_ptr, int O);
	Mat_t gray2bgr(Mat_t img);
	cv_bridge::CvImagePtr mat_left;
	cv_bridge::CvImagePtr mat_right;

private:
	ros::Timer disp_timer;
	ros::NodeHandle nh_;
	ros::Publisher ObjLocationPub;
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber image_left_;
	image_transport::CameraSubscriber image_right_;
	image_transport::Publisher image_pub_;


	sensor_msgs::Image left_image;
	sensor_msgs::Image right_image;
	sensor_msgs::CameraInfo left_info;
	sensor_msgs::CameraInfo right_info;
	std::string cascade_path_WHA,
				cascade_path_PINK;
	CascadeClassifier_t cascade_WHA, cascade_PINK;
	tf::TransformListener optimus_prime;
	object_locator::DetectionManager sherlock;

	typedef std::pair<int, int> PixPoint_t;
	typedef std::pair<PixPoint_t, object_type> Detection_t;
	typedef boost::shared_ptr<Detection_t> DetectionPtr_t;
	std::vector<DetectionPtr_t> detection_list_;

	image_geometry::StereoCameraModel stereo_model;
	char* WINDOWLeft;
	char* WINDOWRight;
	char* WINDOWDisparity;
	Mat_t disparity;
	bool gotLeft;
	bool gotRight;
	int ctrLeft,ctrRight;
	bool objset;
	int HuethresH,
	HuethresL,
	SatthresL,
	SatthresH,
	ValthresL,
	ValthresH,
	erosionCount,
	blurSize,CUDA_ENABLED;
};


};



#endif /* ROS2CV_H_ */
