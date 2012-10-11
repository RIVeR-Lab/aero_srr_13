/*
 * Tentacle.h
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#ifndef TENTACLE_H_
#define TENTACLE_H_

#include<ros/ros.h>
#include <boost/geometry.hpp>

class TentacleGenerator {
public:
	TentacleGenerator(int numTentacles, double expFact, std::vector<double>& speedSets);
	virtual ~TentacleGenerator();

	void getTentacle(int speedSet, int index, std::vector<boost::geometry::model::point<int, 2, boost::geometry::cs::cartesian>>& result);

private:
	int 				numTentacles;	///Number of tentacles per speed-set
	double 				expFact;		///Exponential factor used to calculate radii
	std::vector<double> speedSets;		///std::vector<double> containing the seed radius for each speed set. The size of this vector determines the number of speed sets

};
#endif /* TENTACLE_H_ */
