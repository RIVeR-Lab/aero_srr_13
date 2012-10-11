/*
 * Tentacle.cpp
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#include "TentacleGenerator.h"

namespace oryx_path_planner{

TentacleGenerator::TentacleGenerator(int numTentacles, double expFact, std::vector<double>& speedSets):
		tentacles(speedSets.size()){
	this->expFact 		= expFact;
	this->numTentacles	= numTentacles;
	TentacleGenerator::generateTentacles(speedSets);
}

TentacleGenerator::~TentacleGenerator() {
	// TODO Auto-generated destructor stub
}

void TentacleGenerator::getTentacle(int speedSet, int index, std::vector<pair<int> >& result){

}



void TentacleGenerator::generateTentacles(std::vector<double>& speedSets){
	//Build the speed sets
	for(unsigned int v=0; v<speedSets.size(); v++){
		std::vector<std::vector<pair<int> > >& speedSet = this->tentacles.at(v);
		speedSet.resize(numTentacles);
		//Build the tentacles in a speed set
		for(int t=0; t<speedSet.size(); t++){
			std::vector<pair<int> >& tentacle = speedSet.at(t);

		}
	}
}

};
