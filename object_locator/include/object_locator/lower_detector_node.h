/*
 * LOWER_DETECTOR_NODE.h
 *
 *  Created on: Mar 7, 2013
 *      Author: ssr
 */

#ifndef LOWER_DETECTOR_NODE_H_
#define LOWER_DETECTOR_NODE_H_


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
#include <queue>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>





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
	void rectRightCb(const sensor_msgs::ImageConstPtr& msg);
	void rectLeftCb(const sensor_msgs::ImageConstPtr& msg);
	float nNdisp(const cv::Point2d& pt, const Mat_t& disp);
//	void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud);
	void addBbox(Mat_t& img, Mat_t& final);
	Mat_t gray2bgr(Mat_t img);
	cv::Point2f blobIdentify(Mat_t& img, int objThresh);
	cv_bridge::CvImagePtr mat_left;
	cv_bridge::CvImagePtr mat_right;

private:
	ros::Timer disp_timer;
	ros::NodeHandle nh_;
	ros::Publisher ObjLocationPub;
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber image_left_;
	image_transport::CameraSubscriber image_right_;
	ros::Subscriber disp_image_sub_;
	ros::Subscriber left_rect_sub_;
	ros::Subscriber right_rect_sub_;
	ros::Subscriber point_cloud_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher pub_points2_;

	sensor_msgs::Image left_image;
	sensor_msgs::Image right_image;
	sensor_msgs::CameraInfo left_info;
	sensor_msgs::CameraInfo right_info;
	std::string cascade_path_WHA,
				cascade_path_PINK,
				cascade_path_WHASUN,
				cascade_path_RQT_BALL,
				cascade_path_PIPE;
	float kAvgVal_;
	cv::Point2f pipePoint_;

	Mat_t frame;
	CascadeClassifier_t cascade_WHA, cascade_PINK, cascade_WHASUN, cascade_RQT_BALL,cascade_PIPE;
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
	int CUDA_ENABLED;
};


};



#endif /* LOWER_DETECTOR_NODE_H_ */
