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
//******************* LOCAL DEPENDANCIES ****************//

//*********************** NAMESPACES ********************//
namespace aero_odometry
{
/**
 * @brief Linearly interpolates between two transform matrices based on time
 * @param previous
 * @param previous_time
 * @param current
 * @param current_time
 * @param interp_time
 * @param result
 */
void linearInterpolate(const tf::Transform& previous, const ros::Time& previous_time, const tf::Transform& current, const ros::Time& current_time, const ros::Time& interp_time, tf::Transform& result);

/**
 * @brief Linearly interpolates between two transform matrices based on time
 * @param previous
 * @param current
 * @param interp_time
 * @param result
 */
void linearInterpolate(const tf::StampedTransform& previous, const tf::StampedTransform& current, const ros::Time& interp_time, tf::StampedTransform& result);
}; /* END aero_orometry */

#endif /* LINEARINTERPOLATION_HPP_ */
