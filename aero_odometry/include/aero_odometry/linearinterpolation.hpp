/**
 * @file	linearinterpolation.hpp
 * @date	Jan 28, 2013
 * @author	parallels
 * @brief	//TODO fill in detailed discription here
 */

/*
* LICENSE FILE
*/

#ifndef LINEARINTERPOLATION_HPP_
#define LINEARINTERPOLATION_HPP_

//******************* SYSTEM DEPENDANCIES ****************//
#include<ros/ros.h>
#include<tf/tf.h>
#include<geometry_msgs/Pose.h>
//******************* LOCAL DEPENDANCIES ****************//

//*********************** NAMESPACES ********************//
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using ros::Time;
namespace aero_odometry
{
/**
 * @brief Linearly interpolates between two poses based on time
 * @param previous
 * @param previous_time
 * @param current
 * @param current_time
 * @param interp_time
 * @param result
 */
void linearInterpolate(const Pose& previous, const Time& previous_time, const Pose& current, const Time& current_time, const Time& interp_time, Pose& result);

/**
 * @brief Linearly interpolates between two poses based on time
 * @param previous
 * @param current
 * @param interp_time
 * @param result
 */
void linearInterpolate(const PoseStamped& previous, const PoseStamped& current, const Time& interp_time, PoseStamped& result);
}; /* END aero_orometry */

#endif /* LINEARINTERPOLATION_HPP_ */
