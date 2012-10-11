/*
 * Tentacle.cpp
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#include "Tentacles.h"

namespace oryx_path_planner{

TentacleGenerator::TentacleGenerator(int numTentacles, double expFact, double resolution, double xDim, double yDim, std::vector<double>& speedSets):
		speedSets(speedSets.size()){
	this->expFact 		= expFact;
	this->numTentacles	= numTentacles;
	TentacleGenerator::generateTentacles(speedSets);
}

TentacleGenerator::~TentacleGenerator() {
	// TODO Auto-generated destructor stub
}

oryx_path_planner::Tentacle& TentacleGenerator::getTentacle(int speedSet, int index){
	return this->speedSets.at(speedSet).getTentacle(index);
}



void TentacleGenerator::generateTentacles(std::vector<double>& speedSets){
	//Build the speed sets
	for(unsigned int v=0; v<speedSets.size(); v++){

	}
}

};
