/*
 * StereoNode.cpp
 *
 *  Created on: Mar 23, 2013
 *      Author: bpwiselybabu
 */

#include "StereoNode.h"

namespace prosilica {

StereoNode::StereoNode(const ros::NodeHandle& node_handle) : nh_(node_handle),
	      it_(nh_),
	      cam_l_(NULL),
	      cam_r_(NULL),
	      running_(false)
{
	//TODO: set up variables for camera statistics... see the original prosilica driver
	// Two-stage initialization: in the constructor we open the requested camera. Most
	// parameters controlling capture are set and streaming started in configure(), the
    // callback to dynamic_reconfig.

	prosilica::init();

	if (prosilica::numCameras() < 2)
	   ROS_WARN("Found only %d cameras on local subnet", prosilica::numCameras());

	// Determine which camera to use. Opening by IP address only not by guid
	// TODO: use guid to verify that the correct camera is connected to the correct IP address
	// note the inherent assumption that the master is the the left camer ans the right is the slave

	ros::NodeHandle local_nh("~");
	//unsigned long guid_l = 0;
	//unsigned long guid_r = 0;

	std::string ip_str_l,ip_str_r;
	std::string frameid_l,frameid_r;

	if (local_nh.getParam("master_ip", ip_str_l) && !ip_str_l.empty())
	{
	  	cam_l_.reset( new prosilica::Camera(ip_str_l.c_str()) );
	}
	if (local_nh.getParam("slave_ip", ip_str_r) && !ip_str_r.empty())
	{
	    cam_r_.reset( new prosilica::Camera(ip_str_r.c_str()) );
	}
	if (local_nh.getParam("master_frameid", frameid_l) && !frameid_l.empty())
	{
	  	img_l_.header.frame_id = cam_info_l_.header.frame_id=frameid_l;
	}
	if (local_nh.getParam("slave_frameid", frameid_r) && !frameid_r.empty())
	{
		img_l_.header.frame_id = cam_info_l_.header.frame_id=frameid_r;
	}
	double freq;
	if (local_nh.getParam("freq", freq) && (freq!=0))
	{
		freq_=float(freq);
	}
	int exp;
	if (local_nh.getParam("exposure", exp) && (exp!=0))
	{
		exposure_=exp;
	}
    // Record some attributes of the camera
    tPvUint32 dummy;
    PvAttrRangeUint32(cam_l_->handle(), "Width", &dummy, &sensor_width_);
    PvAttrRangeUint32(cam_l_->handle(), "Height", &dummy, &sensor_height_);

    //No need to do if left is equal to right otherwise use this to check

    tPvUint32 sensor_width2,sensor_height2;
    PvAttrRangeUint32(cam_r_->handle(), "Width", &dummy, &sensor_width2);
	PvAttrRangeUint32(cam_r_->handle(), "Height", &dummy, &sensor_height2);

	cam_l_->getAttribute("TimeStampFrequency",clock_l_);
	cam_r_->getAttribute("TimeStampFrequency",clock_r_);

	if((sensor_width2!=sensor_width_)||(sensor_height2!=sensor_height_))
		ROS_WARN("Master and Slave camera dont match in image size");

   // Try to load intrinsics from on-camera memory.
    loadIntrinsics();

    set_camera_info_srv_l_ = nh_.advertiseService("left/set_camera_info", &StereoNode::setCameraInfoL, this);
    set_camera_info_srv_r_ = nh_.advertiseService("right/set_camera_info", &StereoNode::setCameraInfoR, this);

    configure();
}
void StereoNode::configure()
{
	//The master camera needs to be kept at fixed rate and the slave needs to be set at syncin2

	// Exposure
     cam_l_->setExposure(0, prosilica::Auto);
     cam_r_->setExposure(0, prosilica::Auto);
	 tPvUint32 us = exposure_;
     cam_l_->setAttribute("ExposureAutoMax", us);
     cam_r_->setAttribute("ExposureAutoMax", us);

	 cam_l_->setGain(0, prosilica::Auto);
	 cam_r_->setGain(0, prosilica::Auto);

	 cam_l_->setWhiteBalance(0, 0, prosilica::Auto);
	 cam_l_->setWhiteBalance(0, 0, prosilica::Auto);
	 cam_l_->setAttribute("FrameRate",freq_);
	 cam_l_->setAttributeEnum("SyncOut2Invert", "Off");
	 cam_l_->setAttributeEnum("SyncOut2Mode", "Exposing");
      start();
}
void StereoNode::loadIntrinsics()
{
    // Retrieve contents of user memory
    std::string buffer(prosilica::Camera::USER_MEMORY_SIZE, '\0');
    cam_l_->readUserMemory(&buffer[0], prosilica::Camera::USER_MEMORY_SIZE);

    // Parse calibration file
    std::string camera_name;
    if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, cam_info_l_))
      ROS_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
    else
      ROS_WARN("Failed to load intrinsics from camera");

    cam_r_->readUserMemory(&buffer[0], prosilica::Camera::USER_MEMORY_SIZE);

    // Parse calibration file
    if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, cam_info_r_))
      ROS_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
    else
      ROS_WARN("Failed to load intrinsics from camera");
 }

bool StereoNode::setCameraInfoL(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
{
    ROS_INFO("New camera info received");
    sensor_msgs::CameraInfo &info = req.camera_info;

    // Sanity check: the image dimensions should match the max resolution of the sensor.
    tPvUint32 width, height, dummy;
    PvAttrRangeUint32(cam_l_->handle(), "Width", &dummy, &width);
    PvAttrRangeUint32(cam_l_->handle(), "Height", &dummy, &height);
    if (info.width != width || info.height != height) {
      rsp.success = false;
      rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                          "setting, camera running at resolution %ix%i.")
                            % info.width % info.height % width % height).str();
      ROS_ERROR("%s", rsp.status_message.c_str());
      return true;
    }

    stop();

    std::string cam_name = "Master";

    std::stringstream ini_stream;
    if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, cam_name, info)) {
      rsp.status_message = "Error formatting camera_info for storage.";
      rsp.success = false;
    }
    else {
      std::string ini = ini_stream.str();
      if (ini.size() > prosilica::Camera::USER_MEMORY_SIZE) {
        rsp.success = false;
        rsp.status_message = "Unable to write camera_info to camera memory, exceeded storage capacity.";
      }
      else {
        try {
          cam_l_->writeUserMemory(ini.c_str(), ini.size());
          cam_info_l_ = info;
          rsp.success = true;
        }
        catch (prosilica::ProsilicaException &e) {
          rsp.success = false;
          rsp.status_message = e.what();
        }
      }
    }
    if (!rsp.success)
      ROS_ERROR("%s", rsp.status_message.c_str());

    start();

    return true;
}
bool StereoNode::setCameraInfoR(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
{
    ROS_INFO("New camera info received");
    sensor_msgs::CameraInfo &info = req.camera_info;

    // Sanity check: the image dimensions should match the max resolution of the sensor.
    tPvUint32 width, height, dummy;
    PvAttrRangeUint32(cam_r_->handle(), "Width", &dummy, &width);
    PvAttrRangeUint32(cam_r_->handle(), "Height", &dummy, &height);
    if (info.width != width || info.height != height) {
      rsp.success = false;
      rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                          "setting, camera running at resolution %ix%i.")
                            % info.width % info.height % width % height).str();
      ROS_ERROR("%s", rsp.status_message.c_str());
      return true;
    }

    stop();

    std::string cam_name = "slave";

    std::stringstream ini_stream;
    if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, cam_name, info)) {
      rsp.status_message = "Error formatting camera_info for storage.";
      rsp.success = false;
    }
    else {
      std::string ini = ini_stream.str();
      if (ini.size() > prosilica::Camera::USER_MEMORY_SIZE) {
        rsp.success = false;
        rsp.status_message = "Unable to write camera_info to camera memory, exceeded storage capacity.";
      }
      else {
        try {
          cam_r_->writeUserMemory(ini.c_str(), ini.size());
          cam_info_r_ = info;
          rsp.success = true;
        }
        catch (prosilica::ProsilicaException &e) {
          rsp.success = false;
          rsp.status_message = e.what();
        }
      }
    }
    if (!rsp.success)
      ROS_ERROR("%s", rsp.status_message.c_str());

    start();

    return true;
 }

void StereoNode::start()
{
	if (running_) return;

	cam_l_->setFrameCallback(boost::bind(&StereoNode::publishImageL, this, _1));
	cam_r_->setFrameCallback(boost::bind(&StereoNode::publishImageR, this, _1));

	streaming_pub_l_ = it_.advertiseCamera("left/image_raw", 1);
	streaming_pub_r_ = it_.advertiseCamera("right/image_raw", 1);

	//assuming left as the maseter
	//can be multithreaded using boost at this point!!

	cam_l_->startThread(prosilica::FixedRate, prosilica::Continuous);
	cam_r_->startThread(prosilica::SyncIn2, prosilica::Continuous);
	//not there in the GC095
	//cam_l_->runCommand("TimeStampReset");
	//cam_r_->runCommand("TimeStampReset");

    running_ = true;
 }

void StereoNode::stop()
{
   cam_l_->stopThread();
   cam_r_->stopThread();

   cam_l_->stop(); // Must stop camera before streaming_pub_.
   cam_r_->stop();
   //trigger_sub_.shutdown();
   streaming_pub_l_.shutdown();
   streaming_pub_r_.shutdown();
   running_ = false;
}

static bool frameToImage(tPvFrame* frame, sensor_msgs::Image &image)
{
    // NOTE: 16-bit and Yuv formats not supported
    static const char* BAYER_ENCODINGS[] = { "bayer_rggb8", "bayer_gbrg8", "bayer_grbg8", "bayer_bggr8" };

    std::string encoding;
    if (frame->Format == ePvFmtMono8)       encoding = sensor_msgs::image_encodings::MONO8;
    else if (frame->Format == ePvFmtBayer8)
    {
#if 1
      encoding = BAYER_ENCODINGS[frame->BayerPattern];
#else
      image.encoding = sensor_msgs::image_encodings::BGR8;
      image.height = frame->Height;
      image.width = frame->Width;
      image.step = frame->Width * 3;
      image.data.resize(frame->Height * (frame->Width * 3));
      PvUtilityColorInterpolate(frame, &image.data[2], &image.data[1], &image.data[0], 2, 0);
      return true;
#endif
    }
    else if (frame->Format == ePvFmtRgb24)  encoding = sensor_msgs::image_encodings::RGB8;
    else if (frame->Format == ePvFmtBgr24)  encoding = sensor_msgs::image_encodings::BGR8;
    else if (frame->Format == ePvFmtRgba32) encoding = sensor_msgs::image_encodings::RGBA8;
    else if (frame->Format == ePvFmtBgra32) encoding = sensor_msgs::image_encodings::BGRA8;
    else {
      ROS_WARN("Received frame with unsupported pixel format %d", frame->Format);
      return false;
    }

    uint32_t step = frame->ImageSize / frame->Height;
    return sensor_msgs::fillImage(image, encoding, frame->Height, frame->Width, step, frame->ImageBuffer);
  }

 bool StereoNode::processFrameL(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info)
 {
    unsigned long count=frame->FrameCount;
    unsigned long Timestampl=frame->TimestampLo;
    unsigned long Timestamph=frame->TimestampHi;
    uint64_t t=(Timestamph<<32);
    t+=Timestampl;
    double sec = t/double(clock_l_);
    ROS_INFO("Left trig time %f",sec);
    trig_time_l_.fromSec(sec);
    img.header.stamp = cam_info.header.stamp = trig_time_l_;

    if (!frameToImage(frame, img))
      return false;

   return true;
}
 bool StereoNode::processFrameR(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info)
 {
    unsigned long count=frame->FrameCount;
    unsigned long Timestampl=frame->TimestampLo;
    unsigned long Timestamph=frame->TimestampHi;
    uint64_t t=(Timestamph<<32);
    t+=Timestampl;
    double sec= t/double(clock_r_);
    ROS_INFO("right trig time %f",sec);
    trig_time_r_.fromSec(sec);
    img.header.stamp = cam_info.header.stamp = trig_time_r_;

    if (!frameToImage(frame, img))
      return false;

   return true;
}
void StereoNode::publishImageL(tPvFrame* frame)
{

   if (processFrameL(frame, img_l_, cam_info_l_))
      streaming_pub_l_.publish(img_l_, cam_info_l_);
}

void StereoNode::publishImageR(tPvFrame* frame)
{
   if (processFrameR(frame, img_r_, cam_info_r_))
      streaming_pub_r_.publish(img_r_, cam_info_r_);
}

StereoNode::~StereoNode() {
	// TODO Auto-generated destructor stub
}

} /* namespace prosilica */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_prosilica");

  ros::NodeHandle nh("~");
  prosilica::StereoNode pn(nh);
  ros::spin();

  return 0;
}
