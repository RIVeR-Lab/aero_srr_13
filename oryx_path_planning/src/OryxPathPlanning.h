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
 * Provides an implementation of std::exception which allows for chaining of exceptions
 */
class ChainableException: public std::exception{
public:
	/**
	 * Default empty constructor
	 */
	ChainableException():
	message(),
	cause(){
	}
	/**
	 * Standard Copy constructor
	 * @param exception The exception to initialize this exception's fields to
	 */
	ChainableException(oryx_path_planning::ChainableException& exception):
		message(exception.message),
		cause(exception.cause){
	}

	/**
	 * Constructor for creating a new exception without a cause
	 * @param message The error message for this exception
	 */
	ChainableException(std::string& message):
	message(message),
	cause(){
		//this->message += "\nCaused By-> ";
		//this->message += std::string(cause.what());
	}
	/**
	 * Constructor for creating a new exception with a cause
	 * @param message	The error message for the exception
	 * @param cause		The exception which caused this exception
	 */
	ChainableException(std::string& message, std::exception& cause):
	message(message),
	cause(cause){
		this->message += "\nCaused By-> ";
		this->message += std::string(cause.what());
	}

	~ChainableException() throw(){};

	/**
	 * Returns the chained error message in the form of:
	 *
	 * "this_message \n Caused By-> that_message"
	 * @return A char* to a null-terminated character array containing the chained error message
	 */
	virtual const char* what() const throw()
	 {
	    return this->message.c_str();
	 }

	/**
	 * Gets the raw exception that caused this exception
	 * @return A std::exception which caused this exception
	 */
	std::exception& getCause(){
		return this->cause;
	}
protected:
	std::string		message;	///The error message of this ChaingedException
	std::exception	cause;		///The exception which caused this ChaingedException
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
 * @return The location of the point on the grid, equivalent to (int)std::floor(raw*resolution)
 */
inline int roundToGrid(double raw, double resolution){
	return (int)std::floor(raw*resolution);
}

} /* oryx_path_planning */;


#endif /* ORYXPATHPLANNING_H_ */
