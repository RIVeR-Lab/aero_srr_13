/**
 * @file	orxy_path_planning_test.cpp
 * @date	Oct 11, 2012
 * @author	Adam Panzica
 * @brief	Simple test node
 */
#include <ros/ros.h>
#include "Tentacles.h"


int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_path_planning_test");
	ros::NodeHandle nh;
	std::vector<oryx_path_planning::pair<double> > speedSets;
	ROS_INFO("Testing Tentacle Generator...");
	for(int i=0; i<16; i++){
		oryx_path_planning::pair<double> speedData;
		speedData.a = (i+1.0)*5.0;
		speedData.b = 2.0/(16.0-((double)i));
		speedSets.push_back(speedData);
	}
	oryx_path_planning::TentacleGenerator(82, 1.19, .25, 512*.25, 512*.25, speedSets);

	return 0;
}


