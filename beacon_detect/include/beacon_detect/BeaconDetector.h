/*
 * BeaconDetector.h
 *
 *  Created on: May 6, 2013
 *      Author: bpwiselybabu
 */

#ifndef BEACONDETECTOR_H_
#define BEACONDETECTOR_H_
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

class BeaconDetector {
	//variables
	std::string 	cam_topic_;								//the camera topic name
	double 			tag_size_;								//need it so you can calculate the tf
	std::string		tag_frame_prefix_;						//the prefix to the april tag
	std::string 	robot_topic_;							//the topic that will send the robot state message
	cv::Mat			colorImg_;								//the Mat the defines the color image
	std::string 	parent_frame_;							//the name of the parent frame in the tf tree
	cv::Mat			intrinsic_;								//the intrinsic parameter of the camera
	bool			newimg_;								//if the image is new and needs beacon detect to process it

	//ros handles
	tf::TransformBroadcaster 			br_;							//the TF broadcaseter for ROS
	ros::NodeHandle						nh_;							//global node handle;
	image_transport::ImageTransport 	it_;							//the image transport handle
	image_transport::CameraSubscriber 	subImg_;						//the camera subscriber
	image_transport::Publisher 			pub_;							//a image publisher

	//boost thread
	boost::mutex						imglock_;						//mutext to sync the image callback with the detector thread
	boost::thread						detector_thread_;				//boost detector thread handle


	void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);

public:
	//initalization functions
	BeaconDetector();
	/*
	 * Function that will initialize all the ros parameters
	 */
	void getRosParam();

	//beacon detection functions
	/*
	 * this function histogram equalizes the color image
	 */
	void histEq(cv::Mat &frame);
	/**
	 * this function is performs the actual detection of the beacons
	 */
	void detectBeacons();


	virtual ~BeaconDetector();
};

#endif /* BEACONDETECTOR_H_ */
