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
#include<geometry_msgs/Twist.h>
//******************* LOCAL DEPENDANCIES ****************//

//*********************** NAMESPACES ********************//
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using geometry_msgs::TwistStamped;
using ros::Time;
using tf::Quaternion;
using tf::Point;
using tf::Vector3;

namespace aero_odometry
{
/**
 * @author Adam Panzica
 * @brief Linearly interpolates between two poses based on time
 * @param [in]  previous       The pose at the last measurement time
 * @param [in]  previous_time  The ros::Time of the last measurement
 * @param [in]  current        The pose at the current measurement time
 * @param [in]  current_time   The ros::Time of the current measurement
 * @param [in]  interp_time    The ros::Time that should be interpolated to
 * @param [out] result         The resultant interpolated pose
 *
 * Uses standard linear interpolation of the Pose Quaternion and Position to determine the resultant Pose.
 *
 * Note that this function assumes that each pose is relative to the same frame coordinate system
 */
void linearInterpolate(const Pose& previous, const Time& previous_time, const Pose& current, const Time& current_time, const Time& interp_time, Pose& result);

/**
 * @author Adam Panzica
 * @brief Linearly interpolates between two poses based on time
 * @param [in]  previous       The pose and time of the last measurement
 * @param [in]  current        The pose and time of the current measurement
 * @param [in]  interp_time    The ros::Time to interpolate to
 * @param [out] result         The resultant interpolated pose
 *
 * Uses standard linear interpolation of the Pose Quaternion and Position to determine the resultant Pose.
 *
 * Note that the current and previous measurement times are extracted from the geometry_msgs::PoseStamped Header, so those fields
 * must be full for the function to work correctly. The header for the resultant geometry_msgs::PoseStamped is not filled in
 * by this method. Currently, assumes both poses are in the same frame.
 *
 * \todo extend to use tf to allow for poses in different frames to be interpolated to, and for an arbitrary output frame to be
 * specified
 */
void linearInterpolate(const PoseStamped& previous, const PoseStamped& current, const Time& interp_time, PoseStamped& result);

/**
 * @author Adam Panzica
 * @author Mitchell Wills
 * @brief Linearly interpolates between two Twists based on time
 * @param [in]  previous       The Twist at the last measurement time
 * @param [in]  previous_time  The ros::Time of the last measurement
 * @param [in]  current        The Twist at the current measurement time
 * @param [in]  current_time   The ros::Time of the current measurement
 * @param [in]  interp_time    The ros::Time that should be interpolated to
 * @param [out] result         The resultant interpolated Twist
 *
 * Uses standard linear interpolation to determine the resultant Twists.
 *
 * Note that this function assumes that each pose is relative to the same frame coordinate system
 */
void linearInterpolate(const Twist& previous, const Time& previous_time, const Twist& current, const Time& current_time, const Time& interp_time, Twist& result);


/**
 * @author Adam Panzica
 * @author Mitchell Wills
 * @brief Linearly interpolates between two Twists based on time
 * @param [in]  previous       The Twist and time of the last measurement
 * @param [in]  current        The Twist and time of the current measurement
 * @param [in]  interp_time    The ros::Time to interpolate to
 * @param [out] result         The resultant interpolated Twist
 *
 * Uses standard linear interpolation to determine the resultant Twist.
 *
 * Note that the current and previous measurement times are extracted from the geometry_msgs::TwistStamped Header, so those fields
 * must be full for the function to work correctly. The header for the resultant geometry_msgs::TwistStamped is not filled in
 * by this method. Currently, assumes both poses are in the same frame.
 *
 * \todo extend to use tf to allow for Twists in different frames to be interpolated to, and for an arbitrary output frame to be
 * specified
 */
void linearInterpolate(const TwistStamped& previous, const TwistStamped& current, const Time& interp_time, TwistStamped& result);
}; /* END aero_orometry */

#endif /* LINEARINTERPOLATION_HPP_ */
