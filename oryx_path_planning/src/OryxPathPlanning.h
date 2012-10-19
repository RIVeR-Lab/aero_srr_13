/**
 * @file	OryxPathPlanning.h
 * @date	Oct 17, 2012
 * @author	Adam Panzica
 * @brief	Header containing definitions that are used throughout the oryx_path_planning package
 */

#ifndef ORYXPATHPLANNING_H_
#define ORYXPATHPLANNING_H_

#include <ros/ros.h>

namespace oryx_path_planning{

const double PI = std::atan(1.0)*4;	///Since C++ lacks a predefined PI constant, define it here

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

} /* oryx_path_planning */;


#endif /* ORYXPATHPLANNING_H_ */
