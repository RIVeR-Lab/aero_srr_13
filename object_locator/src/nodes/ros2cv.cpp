#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <object_locator/ros2cv.h>
#include <opencv2/opencv.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;



ImageConverter::ImageConverter()
    : it_(nh_),
      WINDOWLeft ("Left Camera"),
      WINDOWRight ("Right Camera"),
	  WINDOWDisparity ("Disparity"),
	  gotLeft(false),
	  gotRight(false)

  {
    image_pub_ = it_.advertise("/out", 1);
    image_left_ = it_.subscribe("/stereo_bottom/left/image_raw", 1, &ImageConverter::imageCbLeft, this);
    image_right_ = it_.subscribe("/stereo_bottom/right/image_raw", 1, &ImageConverter::imageCbRight, this);
    disp_timer = nh_.createTimer(ros::Duration(1/18), &ImageConverter::computeDisparityCb,this);

    cv::namedWindow(WINDOWLeft);
    cv::namedWindow(WINDOWRight);
  }

ImageConverter::~ImageConverter()
  {
    cv::destroyWindow(WINDOWLeft);
    cv::destroyWindow(WINDOWRight);
  }

  void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));


    cv::Mat img;
    img = cv_ptr->image;
    cv::imshow(WINDOW, img);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void ImageConverter::imageCbLeft(const sensor_msgs::ImageConstPtr& msg)
  {
	  ImageConverter::imageCb(msg, this->mat_left, this->WINDOWLeft);
	  gotLeft = true;
  }
  void ImageConverter::imageCbRight(const sensor_msgs::ImageConstPtr& msg)
  {
	  ImageConverter::imageCb(msg, this->mat_right, this->WINDOWRight);
	  gotRight = true;
  }
  void ImageConverter::computeDisparity()
  {
	  cv::StereoVar stereo;
	  stereo.maxDisp = 1500;
	  stereo.minDisp = 300;
	  ROS_INFO_STREAM("processing disparity");
	  stereo(this->mat_left->image, this->mat_right->image, this->disparity);
	  ROS_INFO_STREAM("disparity processed");
	  cv::imshow(WINDOWDisparity, this->disparity);
	  ROS_INFO_STREAM("written to window");
	  cv::waitKey(3);

  }
  void ImageConverter::computeDisparityCb(const ros::TimerEvent& event)
  {
	  if (gotLeft && gotRight)
	  {
		  computeDisparity();
		  gotLeft = false;
		  gotRight = false;
	  }
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
