/*
 * StereoNode.h
 *
 *  Created on: Mar 23, 2013
 *      Author: bpwiselybabu
 */

#ifndef STEREONODE_H_
#define STEREONODE_H_
// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <polled_camera/publication_server.h>

// Diagnostics
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <self_test/self_test.h>

// Dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <driver_base/SensorLevels.h>
//#include "prosilica_camera/ProsilicaCameraConfig.h"

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>

#include "prosilica/prosilica.h"
#include "prosilica/rolling_sum.h"

namespace prosilica {

class StereoNode {

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::CameraPublisher streaming_pub_l_,streaming_pub_r_;

	ros::ServiceServer set_camera_info_srv_l_,set_camera_info_srv_r_;

	// Camera
	boost::thread LeftCamThread_,RightCamThread_;
	boost::scoped_ptr<prosilica::Camera> cam_l_, cam_r_;
	bool running_;
	tPvUint32 sensor_width_, sensor_height_; // full resolution dimensions (maybe should be in lib)
	tPvUint32 clock_l_,clock_r_;
	tPvFloat32 freq_;

	// Hardware triggering
	ros::Time trig_time_l_,trig_time_r_;

	// ROS messages
	sensor_msgs::Image img_l_,img_r_;
	sensor_msgs::CameraInfo cam_info_l_,cam_info_r_;


public:
	StereoNode(const ros::NodeHandle& node_handle);
	void loadIntrinsics();
	bool setCameraInfoL(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);
	bool setCameraInfoR(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);
	void start();
	void configure();
	void publishImageL(tPvFrame* frame);
	void publishImageR(tPvFrame* frame);
	bool processFrameL(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info);
	bool processFrameR(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info);
	void stop();
	virtual ~StereoNode();
};

} /* namespace prosilica */
#endif /* STEREONODE_H_ */
