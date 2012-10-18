/*
 * Tentacle.h
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#ifndef TENTACLE_H_
#define TENTACLE_H_

#include<ros/ros.h>
#include <boost/lexical_cast.hpp>
#include<tf/transform_datatypes.h>
#include"OryxPathPlannerConfig.h"
#include"OryxPathPlanning.h"


namespace oryx_path_planning{

/**
 * @author	Adam Panzica
 * @brief	Exception to flag when there has been a problem generating a tentacle
 */
class TentacleGenerationException : public oryx_path_planning::ChainableException{
public:
	/**
	 * @author Adam Panzica
	 * @brief Constructor for creating a new exception
	 * @param index		The index of the tentacle that had the exception
	 * @param seedRad	The seed radius for the tentacle
	 * @param velocity	The velocity of the tentacle
	 * @param message	The error message describing what went wrong
	 */
	TentacleGenerationException(int index, double seedRad, double velocity, std::string& message):
		oryx_path_planning::ChainableException(generateMessage(index, seedRad, velocity, message)){

	}

	/**
	 * @author Adam Panzica
	 * @brief Constructor for creating a new exception with an underlying exception that caused it
	 * @param index		The index of the tentacle that had the exception
	 * @param seedRad	The seed radius for the tentacle
	 * @param velocity	The velocity of the tentacle
	 * @param message	The error message describing what went wrong
	 * @param cause		The exception which caused this exception
	 */
	TentacleGenerationException(int index, double seedRad, double velocity, std::string& message, std::exception& cause):
		oryx_path_planning::ChainableException(generateMessage(index, seedRad, velocity, message), cause){
	}

private:
	std::string& generateMessage(int index, double seedRad, double velocity, std::string& message){
		message = 	"Tentacle <" +
				boost::lexical_cast<std::string>(index)+
				"> with parameters <" +
				boost::lexical_cast<std::string>(seedRad)+
				", "+
				boost::lexical_cast<std::string>(velocity)+
				"> Had The following Error: "+
				message;
		return message;
	}
};

/**
 * @author	Adam Panzica
 * @brief	Simple accessor exception that provides some debugging details
 */
class TentacleAccessException: public oryx_path_planning::ChainableException{
public:
	/**
	 * Default constructor
	 */
	TentacleAccessException(){};
	/**
	 * @author Adam Panzica
	 * @param indexTried	The index of the tentacle that was attempted to be accessed
	 * @param speedSetIndex	The index of the speed set the tentacle is in
	 */
	TentacleAccessException(int indexTried, int speedSetIndex){
		this->message = "Tried to access Invalid Tentacle <"+ boost::lexical_cast<std::string>(indexTried)+ "> in Speed Set "+ boost::lexical_cast<std::string>(speedSetIndex);
	}

	/**
	 * @author Adam Panzica
	 * @param indexTried	The index of the tentacle that was attempted to be accessed
	 * @param speedSetIndex	The index of the speed set the tentacle is in
	 * @param message		Explination for what happened
	 * @param cause			Exception which caused this exception
	 */
	TentacleAccessException(int indexTried, int speedSetIndex,std::string& message, std::exception& cause):
	oryx_path_planning::ChainableException(setUpMessage(indexTried, speedSetIndex, message), cause){
	}

	~TentacleAccessException() throw(){};
private:
	std::string& setUpMessage(int indexTried, int speedSetIndex, std::string& message){
		message = "Tried to access Invalid Tentacle <"+ boost::lexical_cast<std::string>(indexTried)+ "> in Speed Set <"+ boost::lexical_cast<std::string>(speedSetIndex)+"> which caused: "+message;
		return message;
	}
};

/**
 * @author	Adam Panzica
 * @brief	Simple accessor exception that provides some debugging details
 */
class SpeedSetAccessException: public oryx_path_planning::ChainableException{
public:
	/**
	 * @author Adam Panzica
	 * @param indexTried	The index of the speed set that was attempted to be accessed
	 */
	SpeedSetAccessException(int indexTried){
		this->message = "Tried to access Invalid Speed Set"+ boost::lexical_cast<std::string>(indexTried);
	}

	~SpeedSetAccessException() throw(){};
};

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
	 * @throw TentacleGenerationException If there is a problem generating the tentacle
	 *
	 */
	Tentacle(double expFact, double seedRad, int index, int numTent, double resolution, double xDim, double yDim, double velocity) throw (TentacleGenerationException);
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
	const static double straightThreshold = 1500;	///Cuttoff radius for what is considered to be essentially a straight line

	/**
	 * @author Adam Panzica
	 * @brief Helper function which caluclates the x/y coord of a point on an arc with an offset
	 * @param radius	Radius of the arc
	 * @param theta		Polar theta
	 * @param scale		Amount to scale the results by
	 * @param rshift	amount to shift the result in the x-axis
	 * @param result	Reference to a tf::Point to write the result to
	 */
	//void calcCoord(double radius, double theta, double scale, double rshift, tf::Point& result);
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
	 * @throw TentacleAccessException if the tentacle index was invalid
	 */
	Tentacle& getTentacle(int index) throw(oryx_path_planning::TentacleAccessException);

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
	 * @param minSpeed		The speed of the slowest speed set
	 * @param maxSpeed		The speed of the fastest speed set
	 * @param numSpeedSet	The number of speed sets to generate
	 * @param numTentacles	The number of tentacles in each speed set
	 * @param expFact		The exponential factor used to determine radius for each tentacle
	 * @param resolution	The resolution of the occupancy grid that the tentacles will be overlaid on
	 * @param xDim			The length of the x-axis of the occupancy grid, in the positive x-direction and of the same units as resolution
	 * @param yDim			The length of the y-axis of the occupancy grid, in the positive y-direction and of the same units as resolution
	 */
	TentacleGenerator(double minSpeed, double maxSpeed, int numSpeedSet, int numTentacles, double expFact, double resolution, double xDim, double yDim);
	virtual ~TentacleGenerator();

	/**
	 * @author Adam Panzics
	 * @brief Gets the number of speed sets that were generated
	 * @return The number of speed sets that were generated
	 */
	int getNumSpeedSets();


	/**
	 * @author Adam Panzica
	 * @brief Gets the tentacle from the given speed set and tentacle index
	 * @param speedSet	The speed set to look at
	 * @param index		The index of the tentacle to get
	 * @return A Tentacle containing all of the data about the requested tentacle
	 * @throw TentacleAccessException if the tentacle index was invalid
	 * @throw SpeedSetAccessException if the speed set index was invalid
	  */
	Tentacle& getTentacle(int speedSet, int index) throw(oryx_path_planning::TentacleAccessException, oryx_path_planning::SpeedSetAccessException);

	/**
	 * @author Adam Panzica
	 * @param speedSet Index of the SpeedSet to get
	 * @return The SpeedSet at the index
	 */
	SpeedSet& getSpeedSet(int speedSet);
private:
	int 				numTentacles;	///Number of tentacles per speed-set
	int					numSpeedSet;
	double 				expFact;		///Exponential factor used to calculate radii
	std::vector<SpeedSet > speedSets;	///A set containing all of the valid tentacles that have been generated

	/**
	 * @author Adam Panzics
	 * @brief Calculates the 'magic constant' q used in tentacle generation
	 * @param speedSet The index of the current speed set
	 * @return A constant, @f$ q= \frac{speedSet}{numSpeedSet-1} @f$
	 */
	double calcQ(int speedSet);

	/**
	 * @author Adam Panzica
	 * @brief Helper function which calculates the seed radius for a speed set
	 * @param speedSet The index number of the speed set
	 * @param q			Calculated constant
	 * @return The calculated seed radius
	 */
	double calcSeedRad(int speedSet, double q);

	/**
	 * @author Adam Panzica
	 * @brief Helper function which calculates the velocity for a speed set
	 * @param minSpeed	Minimum speed of all speed sets
	 * @param maxSpeed	Maximum speed of all speed sets
	 * @param q			Calculated constant
	 * @return The velocity for a speed set
	 */
	double calcSpeedSetVel(double minSpeed, double maxSpeed, double q);
};

};
#endif /* TENTACLE_H_ */
