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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

/*Headers Aero */
#include <actionlib/client/simple_action_client.h>
#include <device_driver_base/SetJointPositionAction.h>

#include <boom_face_track/kalman.h>
#include <tf/transform_listener.h>

class FaceTrack
{

	typedef actionlib::SimpleActionClient<device_driver_base::SetJointPositionAction> BoomClient;
	//variables
	std::string 	cam_topic_;								//the camera topic name
	cv::Mat			colorImg_;								//the Mat the defines the color image

	ros::NodeHandle						nh_;							//global node handle;
	image_transport::ImageTransport 	it_;							//the image transport handle
	image_transport::CameraSubscriber 	subImg_;						//the camera subscriber
	ros::ServiceClient 					state_client_;					//the client responsible for make state change of the robot
	boost::shared_ptr<BoomClient> 		boom_client_;					//the action server for controlling the boom

	//boost thread
	boost::thread							world_broadcaster_;				//thread that does tf broadcast of the world

	tf::TransformListener				tf_lr_;							//the TF listener so the node can extract the robot tf tree

	kalman filter_;	
	void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);

       ros::Timer timer_;
public:
	//initalization functions
	FaceTrack();
	/*
	 * Function that will initialize all the ros parameters
	 */
	void getRosParam();
	/*
	 * This function is used to extract the constellation information from the config file
	 */
	void histEq(cv::Mat &frame);
	/**
	 * this function is performs the actual detection of the beacons
	 */
	
	void timerCallback(const ros::TimerEvent& event);
	virtual ~FaceTrack();
};

#endif /* BEACONDETECTOR_H_ */
