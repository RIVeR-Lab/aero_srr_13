/*
 * beaconNode.cpp
 *
 *  Created on: May 6, 2013
 *      Author: bpwiselybabu
 */

#include <ros/ros.h>
#include <beacon_detect/BeaconDetector.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Beacon_detector");
	BeaconDetector BD;
	ros::spin();
}



