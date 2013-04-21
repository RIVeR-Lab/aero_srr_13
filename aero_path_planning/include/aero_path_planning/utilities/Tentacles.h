/*
 * Tentacle.h
 *
 *  Created on: Oct 11, 2012
 *      Author: parallels
 */

#ifndef TENTACLE_H_
#define TENTACLE_H_
//*********************** SYSTEM DEPENDENCIES ************************************//
#include<ros/ros.h>
#include <boost/lexical_cast.hpp>
#include<tf/transform_datatypes.h>

//*********************** LOCAL DEPENDENCIES ************************************//
#include"AeroPathPlanningUtilities.h"


namespace aero_path_planning
{
//*********************** PROTOTYPES ******************************//
class TentacleGenerationException;
class TentacleAccessException;
class SpeedSetAccessException;
class Tentacle;
class SpeedSet;
class TentacleGenerator;

//*********************** TYPEDEFS ******************************//
///Typedef to allow for convenient sharing of a Tentacle via shared pointer
typedef boost::shared_ptr<Tentacle> TentaclePtr;

///Typedef to allow for convenient sharing of a SpeedSet via shared pointer
typedef boost::shared_ptr<SpeedSet> SpeedSetPtr;

///Typedef to allow for convenient sharing of a TentacleGenerator via pointer
typedef boost::shared_ptr<TentacleGenerator> TentacleGeneratorPtr;

//*********************** CLASS DEFINITIONS ************************************//
/**
 * @author	Adam Panzica
 * @brief	Exception to flag when there has been a problem generating a tentacle
 */
class TentacleGenerationException : public aero_path_planning::ChainableException
{
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
		aero_path_planning::ChainableException(generateMessage(index, seedRad, velocity, message))
	{

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
		aero_path_planning::ChainableException(generateMessage(index, seedRad, velocity, message), cause)
	{
	}

private:
	std::string& generateMessage(int index, double seedRad, double velocity, std::string& message)
	{
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
class TentacleAccessException: public aero_path_planning::ChainableException
{
public:
	/**
	 * Default constructor
	 */
	TentacleAccessException(){};
	/**
	 * @author Adam Panzica
	 * @param indexTried	The index of the tentacle that was attempted to be accessed
	 * @param speedSetIndex	The index of the speed set the tentacle is in
	 * @param message		An optional message discribing what went wrong
	 */
	TentacleAccessException(int indexTried, int speedSetIndex, std::string& message = *(new std::string())):
		aero_path_planning::ChainableException(generateMessage(indexTried, speedSetIndex, message))
	{
	}

	/**
	 * @author Adam Panzica
	 * @param indexTried	The index of the tentacle that was attempted to be accessed
	 * @param speedSetIndex	The index of the speed set the tentacle is in
	 * @param message		Explanation for what happened
	 * @param cause			Exception which caused this exception
	 */
	TentacleAccessException(int indexTried, int speedSetIndex, std::string& message, std::exception& cause):
		aero_path_planning::ChainableException(generateMessage(indexTried, speedSetIndex, message), cause)
	{
	}

	~TentacleAccessException() throw(){};
private:
	std::string& generateMessage(int indexTried, int speedSetIndex, std::string& message)
	{
		message = "Tried to access Invalid Tentacle <"+ boost::lexical_cast<std::string>(indexTried)+ "> in Speed Set <"+ boost::lexical_cast<std::string>(speedSetIndex)+">: "+message;
		return message;
	}
};

/**
 * @author	Adam Panzica
 * @brief	Simple accessor exception that provides some debugging details
 */
class SpeedSetAccessException: public aero_path_planning::ChainableException
{
public:
	/**
	 * @author Adam Panzica
	 * @param indexTried	The index of the speed set that was attempted to be accessed
	 * @param message		Optional discriptive error message
	 */
	SpeedSetAccessException(int indexTried, std::string& message = *(new std::string())):
		aero_path_planning::ChainableException(generateMessage(indexTried, message))
	{
	}

	~SpeedSetAccessException() throw(){};
private:
	std::string& generateMessage(int indexTried, std::string& message)
	{
		message = "Tried to access Invalid Speed Set "+ boost::lexical_cast<std::string>(indexTried)+": "+ message;
		return message;
	}
};

/**
 * @author Adam Panzica
 * @brief Container class for holding data about a tentacle
 */
class Tentacle{
public:
	///Typedef to allow for convenient naming of tentacle point cloud
	typedef pcl::PointCloud<aero_path_planning::Point> TentaclePointCloud;
	///Typedef to allow for convenient sharing of a vector of points by pointer
	typedef boost::shared_ptr<pcl::PointCloud<aero_path_planning::Point> > TentacleCloudPtr;
	///Typedef pcl::PointCloud<aero_path_planning::Point>::iterator for convenience
	typedef pcl::PointCloud<aero_path_planning::Point>::iterator iterator;
	///Typedef pcl::PointCloud<aero_path_planning::Point>::const_iterator for convenience
	typedef pcl::PointCloud<aero_path_planning::Point>::const_iterator const_iterator;

	/**
	 * @author	Adam Panzics
	 * @brief	Default constructor for creating an uninitialized Tentacle
	 */
	Tentacle();

	/**
	 * @author	Adam Panzica
	 * @brief	Copy constructor
	 * @param Tentacle The Tentacle to copy
	 */
	Tentacle(Tentacle& Tentacle);

	/**
	 * @author	Adam Panzica
	 * @brief	Copy constructor
	 * @param Tentacle The Tentacle to copy
	 */
	Tentacle(const Tentacle& Tentacle);

	/**
	 * @author Adam Panzica
	 * @brief Creates a new Tentacle using the given parameters for its construction
	 * @param exp_fact 		The exponential factor parameter used to calculate the radius of the tentacle
	 * @param seed_rad 		The seed radius for the speed set the tentacle is in
	 * @param min_length    The length of the shortest tentacle in the slowest speed-set
	 * @param seed_length	The seed length that was used to calculate the seed radius
	 * @param index			The tentacle index of this tentacle
	 * @param num_tent		The total number of tentacles in the speed set
	 * @param resolution	The resolution of the occupancy grid that the tentacle will be overlaid on
	 * @param x_dim			The length of the x-axis of the occupancy grid, in the positive x-direction and of the same units as resolution
	 * @param y_dim			The length of the y-axis of the occupancy grid, in the positive y-direction and of the same units as resolution
	 * @param velocity		The velocity that the tentacle is to be traveled at
	 * @throw TentacleGenerationException If there is a problem generating the tentacle
	 *
	 */
	Tentacle(double exp_fact, double seed_rad, double min_length, double seed_length, int index, int num_tent, double resolution, int x_dim, int y_dim, double velocity) throw (TentacleGenerationException);
	virtual ~Tentacle();

	/**
	 * @author	Adam Panzica
	 * @brief	gets The radius data of the tentacle
	 * @return	The radius data of the tentacle
	 */
	double getRad() const;

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the velocity data of the tentacle
	 * @return	The velocity data of the tentacle
	 */
	double getVel() const;

	/**
	 * @author Adam Panzica
	 * @return The index of the tentacle
	 */
	int getIndex() const;

	/**
	 * @author Adam Panzica
	 * @brief Gets the x/y coordinates of all the points long this tentacle
	 * @return A reference to a vector containing a set of pairs which represent the x/y coordinates relative to robot-center
	 */
	const TentaclePointCloud& getPoints() const;

	/**
	 * @author	Adam Panzica
	 * @brief	Gets an iterator reference to the beginning of the tentacle
	 * @return	An iterator reference to the beginning of the tentacle
	 */
	iterator begin();
	/**
	 * @author	Adam Panzica
	 * @brief	Gets an iterator reference to the end of the tentacle
	 * @return	An iterator reference to the end of the tentacle
	 */
	iterator end();

	/**
	 * @author	Adam Panzica
	 * @brief	Gets an iterator reference to the beginning of the tentacle
	 * @return	An iterator reference to the beginning of the tentacle
	 */
	const_iterator cbegin() const;
	/**
	 * @author	Adam Panzica
	 * @brief	Gets an iterator reference to the end of the tentacle
	 * @return	An iterator reference to the end of the tentacle
	 */
	const_iterator cend() const;

	/**
	 * @author Adam Panzica
	 * @brief Helper class for traversing an aero_path_planning::Tentacle
	 */
	class TentacleTraverser
	{
	public:

		/**
		 * @author Adam Panzica
		 * @brief Constructs a new TentacleTraverser over a given Tentacle
		 * @param tentacle The tentacle to traverse over
		 * Note that the traverser assumes that the points contained in the tentacle are in order,
		 * starting at <0,0,0> and going outward along the tentacle
		 */
		TentacleTraverser(Tentacle& tentacle);

		/**
		 * @author Adam Panzica
		 * @brief Constructs a new TentacleTraverser over a given Tentacle
		 * @param tentacle The tentacle to traverse over
		 * Note that the traverser assumes that the points contained in the tentacle are in order,
		 * starting at <0,0,0> and going outward along the tentacle
		 */
		TentacleTraverser(const Tentacle& tentacle);
		/**
		 * Default destructor
		 */
		virtual ~TentacleTraverser();
		/**
		 * @author	Adam Panzica
		 * @brief	Returns true if there are points left in the traversal
		 * @return	true if there are points left in the traversal, else false
		 */
		bool hasNext() const;
		/**
		 * @author Adam Panzica
		 * @brief	Gets the next point in the traversal
		 * @return	The next Point along the tentacle.
		 * Will return the last point in the traversal for subsequent calls after the end of the traversal is reached
		 */
		const aero_path_planning::Point& next();
		/**
		 * @author Adam Panzica
		 * @brief Gets the length traversed thus far along the Tentacle
		 * @return The length traversed so far along the tentacle
		 */
		double lengthTraversed() const;

		/**
		 * @author	Adam Panzica
		 * @brief	Gets the distance traversed between the last two points
		 * @return	The distance between the last two points (change in lengthTraversed() after last next() call)
		 */
		double deltaLength() const;

	private:

		bool		empty_;			///True if the iterator has reached the end of the traversal
		double		length_;			///The current length traversed along the Tentacle
		double		delta_length_;	///The difference between the current length and last length traversed
		const aero_path_planning::Point*	last_point_;	///Pointer to the last point that was passed
		const aero_path_planning::Point*	next_point_;	///Pointer to the next point that will be passed
		Tentacle::const_iterator start_;	///The start of an iterator over all the Points along the Tentacle
		Tentacle::const_iterator end_;		///The end of the iterator of the points along the tentacle
	};

	///Typedef to allow for convenient sharing of a TentacleTraverser via pointer
	typedef boost::shared_ptr<TentacleTraverser> TentacleTraverserPtr;

private:
	int    index_;
	double radius_;									///Radius of the Tentacle
	double velocity_;								///Velocity of the Tentacle
	TentaclePointCloud points_; 						///A vector containing a set of Points which represent the x/y coordinates relative to robot-center that this tentacle touches
	const static double straight_threshold_ = 2000;	///Cutoff radius for what is considered to be essentially a straight line

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
class SpeedSet
{
public:
	/**
	 * typedef over std::vector<Tentacle>::iterator to allow SpeedSet to return an iterator over the Tentacles it contains
	 */
	typedef std::vector<Tentacle>::iterator iterator;

	/**
	 * typedef over std::vector<Tentacle>::const_iterator to allow SpeedSet to return an iterator over the Tentacles it contains
	 */
	typedef std::vector<Tentacle>::const_iterator const_iterator;

	/**
	 * @author Adam Panzics
	 * @brief Default constructor which creates an empty speed set
	 */
	SpeedSet();

	/**
	 * @author	Adam Panzica
	 * @brief	Copy constructor
	 * @param SpeedSet SpeedSet to copy
	 */
	SpeedSet(const SpeedSet& SpeedSet);

	/**
	 * @author Adam Panzica
	 * @brief Generates all of the tentacles with the given parameters
	 * @param index         The index of the speed set
	 * @param min_length    The length of the shortest tentacle in the slowest speed-set
	 * @param expFact 		The exponential factor parameter used to calculate the radius of the tentacle
	 * @param seedRad 		The seed radius for the speed set
	 * @param seedLength	The seed length that was used to calculate the seed radius
	 * @param numTent		The total number of tentacles in the speed set
	 * @param resolution	The resolution of the occupancy grid that the tentacles will be overlaid on
	 * @param xDim			The length of the x-axis of the occupancy grid, in the positive x-direction and of the same units as resolution
	 * @param yDim			The length of the y-axis of the occupancy grid, in the positive y-direction and of the same units as resolution
	 * @param velocity		The velocity that the tentacle is to be traveled at
	 */
	SpeedSet(int index, double min_length, double expFact, double seedRad, double seedLength, int numTent, double resolution, int xDim, int yDim, double velocity);
	virtual ~SpeedSet();

	/**
	 * @author Adam Panzica
	 * @brief Gets a Tentacle from the speed set
	 * @param index The index of the tentacle to get
	 * @return A reference to a Tentacle from the speed set
	 * @throw TentacleAccessException if the tentacle index was invalid
	 */
	const Tentacle& getTentacle(int index) const throw(aero_path_planning::TentacleAccessException);

	/**
	 * @author Adam Panzica
	 * @brief Gets the number of tentacles in the SpeedSet
	 * @return The number of tentacles in the SpeedSet
	 */
	unsigned int getNumTentacle() const;

	/**
	 * @author Adam Panzica
	 * @return The index of the speed set
	 */
	int getIndex() const;

	/**
	 * @author	Adam Panzica
	 * @return	An iterator pointing to the first index Tentacle in the SpeedSet
	 */
	iterator begin();

	/**
	 * @author	Adam Panzica
	 * @return	An iterator pointing to the last index Tentacle in the SpeedSet
	 */
	iterator end();

	/**
	 * @author	Adam Panzica
	 * @return	An iterator pointing to the first index Tentacle in the SpeedSet
	 */
	const_iterator cbegin() const;

	/**
	 * @author	Adam Panzica
	 * @return	An iterator pointing to the last index Tentacle in the SpeedSet
	 */
	const_iterator cend() const;

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the velocity of this SpeedSet
	 * @return	The velocity value of this SpeedSet
	 */
	double getVelocity() const;

	/**
	 * @author	Adam Panzica
	 * @brief	Gets the seed radius of this SpeedSet
	 * @return	The calculated sseed radius of this SpeedSet
	 */
	double getSeedRad() const;


private:
	int    index_;
	double velocity_;
	double seed_rad_;
	std::vector<Tentacle> tentacles_;	///A vector containing all of the tentacles for this speed set
};

/**
 * @author	Adam Panzics
 * @brief	Generates and manages tentacle data
 */
class TentacleGenerator
{
public:
	/**
	 * typedef over std::vector<SpeedSet>::iterator for convenience
	 */
	typedef std::vector<SpeedSetPtr>::iterator iterator;
	/**
	 * typedef over std::vector<SpeedSet>::const_iterator for convenience
	 */
	typedef std::vector<SpeedSetPtr>::const_iterator const_iterator;

	/**
	 * default empty constructor
	 */
	TentacleGenerator();

	/**
	 * @author	Adam Panzica
	 * @brief	Copy Constructor
	 * @param TentacleGenerator TentacleGenerator to copy
	 */
	TentacleGenerator(TentacleGenerator& TentacleGenerator);

	/**
	 * @author	Adam Panzica
	 * @brief	Copy Constructor
	 * @param TentacleGenerator TentacleGenerator to copy
	 */
	TentacleGenerator(const TentacleGenerator& TentacleGenerator);


	/**
	 * @author Adam Panzica
	 * @brief Generates a set of tentacles for each speed set
	 * @param min_length    The length of the shortest tentacle in the slowest speed-set
	 * @param minSpeed		The speed of the slowest speed set
	 * @param maxSpeed		The speed of the fastest speed set
	 * @param numSpeedSet	The number of speed sets to generate
	 * @param numTentacles	The number of tentacles in each speed set
	 * @param expFact		The exponential factor used to determine radius for each tentacle
	 * @param resolution	The resolution of the occupancy grid that the tentacles will be overlaid on
	 * @param xDim			The length of the x-axis of the occupancy grid, in the positive x-direction and of the same units as resolution
	 * @param yDim			The length of the y-axis of the occupancy grid, in the positive y-direction and of the same units as resolution
	 */
	TentacleGenerator(double min_length, double minSpeed, double maxSpeed, int numSpeedSet, int numTentacles, double expFact, double resolution, int xDim, int yDim);
	virtual ~TentacleGenerator();

	/**
	 * @author Adam Panzics
	 * @brief Gets the number of speed sets that were generated
	 * @return The number of speed sets that were generated
	 */
	int getNumSpeedSets() const;


	/**
	 * @author Adam Panzica
	 * @brief Gets the tentacle from the given speed set and tentacle index
	 * @param speedSet	The speed set to look at
	 * @param index		The index of the tentacle to get
	 * @return A Tentacle containing all of the data about the requested tentacle
	 * @throw TentacleAccessException if the tentacle index was invalid
	 * @throw SpeedSetAccessException if the speed set index was invalid
	 */
	const Tentacle& getTentacle(int speedSet, int index) const throw(aero_path_planning::TentacleAccessException, aero_path_planning::SpeedSetAccessException);

	/**
	 * @author	Adam Panzica
	 * @brief	Looks up a speed set based on index
	 * @param speedSet Index of the SpeedSet to get
	 * @return The SpeedSet at the index
	 */
	const SpeedSet& getSpeedSet(int speedSet) const;

	/**
	 * @author	Adam Panzica
	 * @brief	Looks up a SpeedSet based on velocity
	 * @param velocity Velocity to find a closest match for
	 * @return A SpeedSetPtr to the SpeedSet who most closely matches the given velocity
	 */
	const SpeedSet& getSpeedSet(double velocity) const;

	/**
	 * @author	Adam Panzica
	 * @return	An iterator pointing to the first index SpeedSet in the TentacleGenerator
	 */
	iterator begin();

	/**
	 * @author	Adam Panzica
	 * @return	An iterator pointing to the last index SpeedSet in the TentacleGenerator
	 */
	iterator end();
	/**
	 * @author	Adam Panzica
	 * @return	An iterator pointing to the first index SpeedSet in the TentacleGenerator
	 */
	const_iterator cbegin() const;

	/**
	 * @author	Adam Panzica
	 * @return	An iterator pointing to the last index SpeedSet in the TentacleGenerator
	 */
	const_iterator cend() const;
private:
	int 				num_tentacles_;	///Number of tentacles per speed-set
	int					num_speed_set_;
	double 				exp_fact_;		///Exponential factor used to calculate radii
	std::vector<SpeedSetPtr > speed_sets_;	///A set containing all of the valid speed sets that have been generated
	std::vector<double>		  velocity_keys_;	///A set containing the velocity keys for each speed set

	/**
	 * @author Adam Panzica
	 * @param q	Calculated constant
	 * @return The calculated length constant for the speed set
	 */
	double calcL(double q);

	/**
	 * @author Adam Panzics
	 * @brief Calculates the 'magic constant' q used in tentacle generation
	 * @param speedSet The index of the current speed set
	 * @return A constant, @f$ q= \frac{speedSet}{numSpeedSet-1} @f$
	 */
	double calcQ(int speedSet) const;

	/**
	 * @author Adam Panzica
	 * @brief Helper function which calculates the seed radius for a speed set
	 * @param speedSet The index number of the speed set
	 * @param q		Calculated constant
	 * @param l		Calculated seed length for the speed set
	 * @return The calculated seed radius
	 */
	double calcSeedRad(int speedSet, double l, double q) const;

	/**
	 * @author Adam Panzica
	 * @brief Helper function which calculates the velocity for a speed set
	 * @param minSpeed	Minimum speed of all speed sets
	 * @param maxSpeed	Maximum speed of all speed sets
	 * @param q			Calculated constant
	 * @return The velocity for a speed set
	 */
	double calcSpeedSetVel(double minSpeed, double maxSpeed, double q) const;
};

};
#endif /* TENTACLE_H_ */
