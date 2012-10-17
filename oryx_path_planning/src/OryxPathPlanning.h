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
inline int expandToGrid(double raw, double resolution){
	return (int)std::floor(raw*resolution);
}

} /* oryx_path_planning */;


#endif /* ORYXPATHPLANNING_H_ */
