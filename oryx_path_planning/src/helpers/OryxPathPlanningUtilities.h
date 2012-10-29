/**
 * @file	OryxPathPlanningUtilities.h
 * @date	Oct 17, 2012
 * @author	Adam Panzica
 * @brief	Header containing definitions that are used throughout the oryx_path_planning package
 */

#ifndef ORYXPATHPLANNINGUTILITIES_H_
#define ORYXPATHPLANNINGUTILITIES_H_

#ifndef POINT_PRECISION
#define POINT_PRECISION float	///Macro for defining the precision of the point values being used
#else
#error POINT_PRECISION is already defined
#endif

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/distances.h>
#include <sensor_msgs/PointCloud2.h>

//*********************** MACROS ************************************//
///Macro for printing out warning messages if default parameters are used
#define PARAM_WARN(param,value) ROS_WARN(warn_message.c_str(), param.c_str(), value.c_str())


namespace oryx_path_planning{

//*********************** TYPEDEFS ******************************//
///Typedef to allow for easier to read code
typedef pcl::PointXYZRGBA Point;

///Typedef to allow for convenient sharing of a pcl::PointXYZRGBA via pointer
typedef boost::shared_ptr<Point> PointPtr;

///Typedef to allow for convenient sharing of a PointCloud<pcl::PointXYZRGBA> > via pointer
typedef boost::shared_ptr<pcl::PointCloud<Point> > PointCloudPtr;

//*********************** CONSTANTS ******************************//

const double PI = std::atan(1.0)*4;	///Since C++ lacks a predefined PI constant, define it here

const std::string warn_message("Parameter <%s> Not Set. Using Default Value <%s>");	///Standard parameter warning message

/**
 * @author Adam Panzics
 * @brief basic chainable exception class
 *
 * Provides an implementation of std::exception which allows for chaining of exceptions.
 * Returns the chained error message in the form of:
 *
 * "this_message \n Caused By-> that_message"
 */
class ChainableException: public std::runtime_error{
public:
	/**
	 * Default empty constructor
	 */
	ChainableException():
		std::runtime_error(""),
		cause(){
	}
	/**
	 * Standard Copy constructor
	 * @param exception The exception to initialize this exception's fields to
	 */
	ChainableException(oryx_path_planning::ChainableException& exception):
		std::runtime_error(exception),
		cause(exception.cause){
	}

	/**
	 * Constructor for creating a new exception without a cause
	 * @param message The error message for this exception
	 */
	ChainableException(std::string& message):
		std::runtime_error(message),
		cause(){
	}
	/**
	 * Constructor for creating a new exception with a cause
	 * @param message	The error message for the exception
	 * @param cause		The exception which caused this exception
	 */
	ChainableException(std::string& message, std::exception& cause):
		std::runtime_error(genMessage(message, cause)),
		cause(cause){
	}

	~ChainableException() throw(){};

	/**
	 * Gets the raw exception that caused this exception
	 * @return A std::exception which caused this exception
	 */
	std::exception& getCause(){
		return this->cause;
	}
protected:
	std::exception	cause;		///The exception which caused this ChaingedException

	std::string& genMessage(std::string& message, std::exception& cause){
		message += "\nCaused By-> ";
		message += std::string(cause.what());
		return message;
	}
};

/**
 * Rounds a value to the nearest scale point. EX: raw=12.35, frac=.25, return = 12.25
 * @param raw Raw value to round
 * @param frac Fraction to scale to
 * @return The result of std::floor(raw/scale)*scale
 */
inline double roundToFrac(double raw, double frac){
	return std::floor(raw/frac)*frac;
}

/**
 * Turns a real coordinate into an integer value for a grid with a given resolution.
 * @param raw The raw coordinate value
 * @param resolution The resolution of the grid to place the point on
 * @return The location of the point on the grid, equivalent to (int)std::floor(raw/resolution)
 */
inline int roundToGrid(double raw, double resolution){
	return (int)std::floor(raw/resolution);
}

/**
 * Turns a given point in grid coordinates into a real value
 * @param grid			The point on the grid
 * @param resolution	The resolution of the grid
 * @return A double corresponding to a real value represented by a grid coordinate
 */
inline double gridToReal(int grid, double resolution){
	return (double)grid*resolution;
}

} /* oryx_path_planning */;


#endif /* ORYXPATHPLANNINGUTILITIES_H_ */
