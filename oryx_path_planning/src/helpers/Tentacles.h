/*
 * Tentacle.h
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#ifndef TENTACLE_H_
#define TENTACLE_H_

#include<ros/ros.h>
#include<tf/transform_datatypes.h>
#include"OryxPathPlannerConfig.h"

#if oryx_path_planner_VERBOSITY
#define PRINTER ROS_INFO
#else
#define PRINTER ROS_DEBUG
#endif

namespace oryx_path_planning{
const double PI = std::atan(1.0)*4;	///Since C++ lacks a predefined PI constant, define it here


/**
 * @author Adam Panzica
 * @brief Container class for holding data about a tentacle
 */
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
	tf::Point& getRadVel();

	/**
	 * @author Adam Panzica
	 * @brief Gets the x/y coordinates of all the points long this tentacle
	 * @return A reference to a vector containing a set of pairs which represent the x/y coordinates relative to robot-center
	 */
	std::vector<tf::Point >& getPoints();
private:
	double radius;
	double velocity;
	std::vector<tf::Point > points; 				///A vector containing a set of Points which represent the x/y coordinates relative to robot-center that this tentacle touches
	const static double straightThreshold = 1000;	///Cuttoff radius for what is considered to be essentially a straight line

	/**
	 * @author Adam Panzica
	 * @brief Helper function which caluclates the x/y coord of a point on an arc with an offset
	 * @param radius	Radius of the arc
	 * @param theta		Polar theta
	 * @param scale		Amount to scale the results by
	 * @param rshift	amount to shift the result in the x-axis
	 * @param result	Reference to a tf::Point to write the result to
	 */
	void calcCoord(double radius, double theta, double scale, double rshift, tf::Point& result);
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
	Tentacle& getTentacle(int index);

	/**
	 * @author Adam Panzica
	 * @brief Gets the number of tentacles in the SpeedSet
	 * @return The number of tentacles in the SpeedSet
	 */
	unsigned int getNumTentacle();

private:
	std::vector<Tentacle> tentacles;	///A vector containing all of the tentacles for this speed set
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
	 * @param speedSets		A reference to a vector containing the velocities of each speed set
	 */
	TentacleGenerator(int numTentacles, double expFact, double resolution, double xDim, double yDim, std::vector<double >& speedSets);
	virtual ~TentacleGenerator();

	/**
	 * @author Adam Panzica
	 * @brief Gets the tentacle from the given speed set and tentacle index
	 * @param speedSet	The speed set to look at
	 * @param index		The index of the tentacle to get
	 * @return A Tentacle containing all of the data about the requested tentacle
	 */
	Tentacle& getTentacle(int speedSet, int index);

	/**
	 * @author Adam Panzica
	 * @param speedSet Index of the SpeedSet to get
	 * @return The SpeedSet at the index
	 */
	SpeedSet& getSpeedSet(int speedSet);
private:
	int 				numTentacles;	///Number of tentacles per speed-set
	double 				expFact;		///Exponential factor used to calculate radii
	std::vector<SpeedSet > speedSets;	///A set containing all of the valid tentacles that have been generated

	/**
	 * @author Adam Panzica
	 * @brief Helper function which calculates the seed radius for a speed set
	 * @param speedSet The index number of the speed set
	 * @param numSpeedSet The total number of speed sets
	 * @return The calculated seed radius
	 */
	double calcSeedRad(int speedSet, int numSpeedSet);
};

};
#endif /* TENTACLE_H_ */
