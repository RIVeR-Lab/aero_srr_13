/*
 * Tentacle.cpp
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#include "Tentacles.h"

namespace oryx_path_planner{
//***************************** TENTACLE *********************************//

//***************************** SPEED SET *********************************//
/**
 * This constructor just makes an empty set of tentacles
 */
SpeedSet::SpeedSet(){
	this->tentacles();
}

SpeedSet::SpeedSet(double expFact, double seedRad, int numTent, double resolution, double xDim, double yDim, double velocity){
	ROS_INFO("Generating a Speed Set with the Parameters <SRad=%f, Vel=%f, NumTent=%d, expF=%f>");
	this->tentacles();
	for(unsigned int t=0; t<numTent; t++){
		this->tentacles.push_back(oryx_path_planner::Tentacle(expFact, seedRad, t, numTent, resolution, xDim, yDim, velocity));
	}
}

oryx_path_planner::Tentacle& SpeedSet::getTentacle(int index){
	return this->tentacles.at(index);
}
//***************************** TENTACLE GENERATOR *********************************//
TentacleGenerator::TentacleGenerator(int numTentacles, double expFact, double resolution, double xDim, double yDim, std::vector<pair<double> >& speedSets){
	this->expFact 		= expFact;
	this->numTentacles	= numTentacles;
	this->speedSets();
	ROS_INFO("Generating Speed Sets...");
	//Generate the SpeedSets
	for(unsigned int v=0; v<speedSets.size(); v++){
		this->speedSets.push_back(oryx_path_planner::SpeedSet(expFact, speedSets.at(v).a, numTentacles, resolution, xDim, yDim, speedSets.at(v).b));
	}
	ROS_INFO("Speed Sets Complete!");
}

TentacleGenerator::~TentacleGenerator() {
	// TODO Auto-generated destructor stub
}

oryx_path_planner::Tentacle& TentacleGenerator::getTentacle(int speedSet, int index){
	return this->speedSets.at(speedSet).getTentacle(index);
}

};
