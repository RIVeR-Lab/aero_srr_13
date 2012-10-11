/*
 * Tentacle.cpp
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#include "TentacleGenerator.h"

TentacleGenerator::TentacleGenerator(int numTentacles, double expFact, std::vector<double>& speedSets) {
	this->expFact 		= expFact;
	this->numTentacles	= numTentacles;
	this->speedSets		= speedSets;
}

TentacleGenerator::~TentacleGenerator() {
	// TODO Auto-generated destructor stub
}

void TentacleGenerator::getTentacle(int speedSet, int index, std::vector<boost::geometry::model::point<int, 2, boost::geometry::cs::cartesian>>& result){
	double seedRad = this->speedSets.at(speedSet);
}
