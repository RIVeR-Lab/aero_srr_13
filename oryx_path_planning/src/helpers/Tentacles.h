/*
 * Tentacle.h
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#ifndef TENTACLE_H_
#define TENTACLE_H_

#include<ros/ros.h>
#include"OryxPathPlannerConfig.h"
namespace oryx_path_planner{

/**
 * @author Adam Panzics
 * @brief Simple typedef for defining a basic pair of values
 */
template <class T>
struct pair{
	T a;	///first value of the pair
	T b;	///second value of the pair
};

class Tentacle{
public:
	/**
	 * @author	Adam Panzics
	 * @brief	Default constructor for creating an uninitialized Tentacle
	 */
	Tentacle();
	/**
	 * @author Adam Panzica
	 * @brief Creates a new Tentacle using the given parameters for its construction
	 * @param expFact 		The exponential factor parameter used to calculate the radius of the tentacle
	 * @param seedRad 		The seed radius for the speed set the tentacle is in
	 * @param index			The tentacle index of this tentacle
	 * @param numTent		The total number of tentacles in the speed set
	 * @param resolution	The resolution of the occupancy grid that the tentacle will be overlaid on
	 * @param xDim			The length of the x-axis of the occupancy grid, in the positive x-direction and of the same units as resolution
	 * @param yDim			The length of the y-axis of the occupancy grid, in the positive y-direction and of the same units as resolution
	 * @param velocity		The velocity that the tentacle is to be traveled at
	 *
	 *
	 */
	Tentacle(double expFact, double seedRad, int index, int numTent, double resolution, double xDim, double yDim, double velocity);
	virtual ~Tentacle();

	/**
	 * @author Adam Panzica
	 * @brief gets the radius/velcoity data of the tentacle
	 * @return A pair where pair.a = radius and pair.b = velocity used to generate the tentacle
	 */
	oryx_path_planner::pair<double>& getRadVel();

	/**
	 * @author Adam Panzica
	 * @brief Gets the x/y coordinates of all the points long this tentacle
	 * @return A reference to a vector containing a set of pairs which represent the x/y coordinates relative to robot-center
	 */
	std::vector<pair<int> >& getPoints();
private:
	oryx_path_planner::pair<double> tentacleData;		///A pair containing the radius and velocity of the tentacle
	std::vector<oryx_path_planner::pair<int> > points; ///A vector containing a set of pairs which represent the x/y coordinates relative to robot-center that this tentacle touches
};

/**
 * @author Adam Panzics
 * @brief Container class for holding tentacle data for a speed set
 */
class SpeedSet{
public:
	/**
	 * @author Adam Panzics
	 * @brief Default constructor which creates an empty speed set
	 */
	SpeedSet();
	/**
	 * @author Adam Panzica
	 * @brief Generates all of the tentacles with the given parameters
	 * @param expFact 		The exponential factor parameter used to calculate the radius of the tentacle
	 * @param seedRad 		The seed radius for the speed set
	 * @param numTent		The total number of tentacles in the speed set
	 * @param resolution	The resolution of the occupancy grid that the tentacles will be overlaid on
	 * @param xDim			The length of the x-axis of the occupancy grid, in the positive x-direction and of the same units as resolution
	 * @param yDim			The length of the y-axis of the occupancy grid, in the positive y-direction and of the same units as resolution
	 * @param velocity		The velocity that the tentacle is to be traveled at
	 */
	SpeedSet(double expFact, double seedRad, int numTent, double resolution, double xDim, double yDim, double velocity);
	virtual ~SpeedSet();

	/**
	 * @author Adam Panzica
	 * @brief Gets a Tentacle from the speed set
	 * @param index The index of the tentacle to get
	 * @return A reference to a Tentacle from the speed set
	 */
	oryx_path_planner::Tentacle& getTentacle(int index);

private:
	std::vector<oryx_path_planner::Tentacle> tentacles;	///A vector containing all of the tentacles for this speed set
};

/**
 * @author	Adam Panzics
 * @brief	Generates and manages tentacle data
 */
class TentacleGenerator {
public:
	/**
	 * @author Adam Panzica
	 * @brief Generates a set of tentacles for each speed set
	 * @param numTentacles	The number of tentacles in each speed set
	 * @param expFact		The exponential factor used to determine radius for each tentacle
	 * @param resolution	The resolution of the occupancy grid that the tentacles will be overlaid on
	 * @param xDim			The length of the x-axis of the occupancy grid, in the positive x-direction and of the same units as resolution
	 * @param yDim			The length of the y-axis of the occupancy grid, in the positive y-direction and of the same units as resolution
	 * @param speedSets		A reference to a vector containing pairs in the format pair.a = seed_radius pair.b = velocity
	 */
	TentacleGenerator(int numTentacles, double expFact, double resolution, double xDim, double yDim, std::vector<pair<double> >& speedSets);
	virtual ~TentacleGenerator();

	/**
	 * @author Adam Panzica
	 * @brief Gets the tentacle from the given speed set and tentacle index
	 * @param speedSet	The speed set to look at
	 * @param index		The index of the tentacle to get
	 * @return A Tentacle containing all of the data about the requested tentacle
	 */
	oryx_path_planner::Tentacle& getTentacle(int speedSet, int index);
private:
	int 				numTentacles;					///Number of tentacles per speed-set
	double 				expFact;						///Exponential factor used to calculate radii
	std::vector<oryx_path_planner::SpeedSet > speedSets;	///A set containing all of the valid tentacles that have been generated
};

}; /*oryx_path_planner*/
#endif /* TENTACLE_H_ */
