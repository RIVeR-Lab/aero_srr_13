#include <ros/ros.h>
#include "prosilica/prosilica.h"
#include <cstdio>

int main(int argc, char** argv)
{
  ros::init(argc,argv, "camera_config");
  prosilica::init();
  ros::NodeHandle nh("~");
  std::string ip, sync;
  if(!nh.getParam("IP",ip))
  {
	ROS_INFO("Please set IP");
	return -1;
  }
	
  if(!nh.getParam("SyncOut2Mode",sync))
  {
	ROS_INFO("Please set SyncOut mode");
	return -1;
  }
  try {
    // Open camera at specified IP address
    prosilica::Camera cam(ip.c_str());
    // Load factory settings
    cam.setAttributeEnum("ConfigFileIndex", "Factory");
    cam.runCommand("ConfigFileLoad");
    // Write settings for signaling exposure
    cam.setAttributeEnum("SyncOut2Invert", "Off");
    cam.setAttributeEnum("SyncOut2Mode", sync.c_str());
    // Save settings to config file 1
    cam.setAttributeEnum("ConfigFileIndex", "1");
    cam.runCommand("ConfigFileSave");
    // Always load config file 1 on power up
    cam.setAttributeEnum("ConfigFilePowerUp", "1");
  }
  catch (const prosilica::ProsilicaException& e) {
    ROS_ERROR("CONFIGURATION FAILED:\n%s\n", e.what());
    return 1;
  }
  ROS_INFO("Configured camera successfully.\n");

  prosilica::fini();
  return 0;
}
