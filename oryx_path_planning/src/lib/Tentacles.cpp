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
