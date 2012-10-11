/*
 * Tentacle.h
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#ifndef TENTACLE_H_
#define TENTACLE_H_

#include<ros/ros.h>

namespace oryx_path_planner{

/**
 * Simple typedef for defining a basic pair of values
 */
template <class T>
struct pair{
	T a;	///first value of the pair
	T b;	///second value of the pair
};

class TentacleGenerator {
public:
	TentacleGenerator(int numTentacles, double expFact, std::vector<double>& speedSets);
	virtual ~TentacleGenerator();

	void getTentacle(int speedSet, int index, std::vector<pair<int> >& result);
private:
	int 				numTentacles;					///Number of tentacles per speed-set
	double 				expFact;						///Exponential factor used to calculate radii
	//std::vector<double> speedSets;						///A list of the seed radii for the varius speed sets
	std::vector<std::vector<std::vector<pair<int> > > > tentacles;	///A set containing all of the valid tentacles that have been generated

	void generateTentacles(std::vector<double>& speedSets);
};

}; /*oryx_path_planner*/
#endif /* TENTACLE_H_ */
