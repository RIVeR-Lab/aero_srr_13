/**
 * @file	linearinterpoloation.cpp
 * @date	Jan 28, 2013
 * @author	parallels
 * @brief	//TODO fill in detailed discription here
 */

/*
 * LICENSE FILE
 */
//******************* SYSTEM DEPENDANCIES ****************//
//******************* LOCAL DEPENDANCIES ****************//
#include<aero_odometry/linearinterpolation.hpp>
//*********************** NAMESPACES ********************//
using namespace aero_odometry;


void aero_odometry::linearInterpolate(const tf::Transform& previous, const ros::Time& previous_time, const tf::Transform& current, const ros::Time& current_time, const ros::Time& interp_time, tf::Transform& result)
{

}

void aero_odometry::linearInterpolate(const tf::StampedTransform& previous, const tf::StampedTransform& current, const ros::Time& interp_time, tf::StampedTransform& result)
{

}

