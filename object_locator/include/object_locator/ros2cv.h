/*
 * ros2cv.h
 *
 *  Created on: Mar 7, 2013
 *      Author: ssr
 */

#ifndef ROS2CV_H_
#define ROS2CV_H_

namespace object_locator
{

class ImageConverter
{

public:
	ImageConverter();
	virtual ~ImageConverter();
	void imageCb(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW);
	void imageCbLeft(const sensor_msgs::ImageConstPtr& msg);
	void imageCbRight(const sensor_msgs::ImageConstPtr& msg);
	void computeDisparity();
	void computeDisparityCb(const ros::TimerEvent& event);


private:
	ros::Timer disp_timer;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_left_;
	image_transport::Subscriber image_right_;
	image_transport::Publisher image_pub_;
	cv_bridge::CvImagePtr mat_left;
	cv_bridge::CvImagePtr mat_right;
	char* WINDOWLeft;
	char* WINDOWRight;
	char* WINDOWDisparity;
	cv::Mat disparity;
	bool gotLeft;
	bool gotRight;
};


};



#endif /* ROS2CV_H_ */
