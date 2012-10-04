/*
 * InertiaCube.h
 *
 *  Created on: Feb 14, 2012
 *      Author: oryx
 */

#ifndef INERTIACUBE_H_
#define INERTIACUBE_H_
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include "Kinematics/isense.h"
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <termios.h>
#include <unistd.h>
#include <string>
#include "OryxManager/IMUConfig.h"



ISD_TRACKER_HANDLE handle;
ISD_STATION_INFO_TYPE station;
ISD_TRACKING_DATA_TYPE data;
sensor_msgs::Imu getIMUData();
void reconfigureCallback(OryxManager::IMUConfig &config, uint32_t level);
int port =1;
double publishRate= 10;

//Inertiacube Parameters
std::string TrackerModel;
int State;
bool Compass;
bool Timestamped;
bool GetInputs;
bool GetAuxInputs;
bool GetCameraData;
int CompassCompensation;
int Enhancement;
int Sensitivity;
int Prediction;
int AngleFormat;
int CoordFrame;
int* TipOffset = new int[3];
int YawBoresight;
int* Boresight = new int[3];






#endif /* INERTIACUBE_H_ */
