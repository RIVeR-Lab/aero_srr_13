/*
 * RosBridge.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#include "beacon_detect/RosBridge.h"
using namespace std;
using namespace cv;
RosBridge::RosBridge(string topic_in, string topic_out):it_(nh_)
{
	file_=topic_in;
	pub_ = it_.advertise(topic_out.c_str(), 1);
	subImg_ = it_.subscribeCamera(topic_in.c_str(), 1, &RosBridge::imageCb, this);
	cout<<"Subscribing to: "<<topic_in.c_str()<<endl;
	cout<<"Publishing to: "<<topic_out.c_str()<<endl;
	frame_=new Mat();
	newimg_=false;
}
RosBridge::RosBridge(string topic_in, Mat *img):it_(nh_)
{
	file_ = topic_in;
	subImg_ = it_.subscribeCamera(topic_in.c_str(), 1, &RosBridge::imageCb, this);
	cout<<"Subscribing to: "<<topic_in.c_str()<<endl;
	frame_=new Mat();
	newimg_=false;
}
RosBridge::RosBridge(Mat *img,string topic_out):it_(nh_)
{
	file_ = "User";
	pub_ = it_.advertise(topic_out.c_str(), 1);
	cout<<"Publishing to: "<<topic_out.c_str()<<endl;
	frame_=img;
}
void RosBridge::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
   *frame_=cv_ptr->image;
   intrinsic_=(cv::Mat_<float>(3,3)<<cam_info->K[0],cam_info->K[1],cam_info->K[2],\
                                    cam_info->K[3],cam_info->K[4],cam_info->K[5],\
				    cam_info->K[6],cam_info->K[7],cam_info->K[8]);
   frame_id_=cv_ptr->header.frame_id;

   newimg_=true;
 }

cv::Mat* RosBridge::getNextFrame()
{
	if(newimg_)
	{
		newimg_=false;
		return(frame_);
	}
	return(NULL);
}
void RosBridge::publishFrame()
{
	char label[200];
	sprintf(label,"Topic: %s",file_.c_str());
	addText(label);
	sprintf(label,"Frame No: %d",frame_no_);
	addText(label);
	sprintf(label,"FPS: %f",fps_);
	addText(label);
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv_ptr->header.stamp=ros::Time::now();
	//note the chanege in coding
	cv_ptr->encoding=sensor_msgs::image_encodings::TYPE_8UC3;
	cv_ptr->image = *frame_;
	pub_.publish(cv_ptr->toImageMsg());
}
RosBridge::~RosBridge()
{
	// TODO Auto-generated destructor stub
}
