/**
 * @file	orxy_path_planning_test.cpp
 * @date	Oct 11, 2012
 * @author	Adam Panzica
 * @brief	Simple test node
 */
#include <ros/ros.h>
#include "Tentacles.h"


int int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_path_planning_test");
	std::vector<oryx_path_planner::pair<double> > speedSets;
	for(int i=0; i<16; i++){
		oryx_path_planner::pair<double> speedData = new oryx_path_planner::pair<double>();
		speedData.a = (i+1)*5;
		speedData.b = 2/(16-i);
		speedSets.push_back(speedData);
	}
	oryx_path_planner::TentacleGenerator(82, 1.19, .25, 512*.25, 512*.25, speedSets);
}


