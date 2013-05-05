/**
 * @file   SupervisorUtilities.h
 *
 * @date   Mar 22, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef SUPERVISORUTILITIES_H_
#define SUPERVISORUTILITIES_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
//************ LOCAL DEPENDANCIES ****************//
#ifndef PARAM_WARN
#define PARAM_WARN(param, value) ROS_WARN_STREAM("Warning, Parameter <"<<param<<"> not set, using default value <"<<value<<">")
#endif
//***********    NAMESPACES     ****************//

namespace aero_srr_supervisor
{

/**
 * @author Adam Panzica
 * @brief  Enum over the states that Supervisor can be in
 */
enum State
{
	ERROR,    //!< ERROR The robot has encountered an error
	STARTUP,  //!< STARTUP The robot is currently in its startup sequence
	MANUAL,   //!< MANUAL The robot is currently in manual control mode
	SEARCHING,//!< SEARCHING The robot is currently exploring the map
	NAVOBJ,   //!< NAVOBJ The robot is currently navigating to an object
	COLLECT,  //!< COLLECT The robot is currently collecting an object of interest
	HOME,     //!< HOME The robot is currently returning to the home platform
	PAUSE,    //!< PAUSE The robot is currently paused
	SHUTDOWN, //!< SHUTDOWN The robot is currently in its shutdown sequence
	SAFESTOP  //!< SAFESTOP The robot is currently in a safe-stop configuration
};

};

#endif /* SUPERVISORUTILITIES_H_ */
