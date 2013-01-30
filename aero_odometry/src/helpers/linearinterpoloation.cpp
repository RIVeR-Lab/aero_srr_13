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


namespace aero_odometry
{
void aero_odometry::linearInterpolate(const Pose& previous, const Time& previous_time, const Pose& current, const Time& current_time, const Time& interp_time, Pose& result)
{
	//extract the position/rotation data
	Quaternion pq(previous.orientation);
	Quaternion cq(current.orientation);
	Point      pp(previous.orientation);
	Point      cp(current.position);
	//Calculate the %change in time
	double scale        = (interp_time-previous_time)/(current_time-previous_time);

	//Linearly interpolate the orientation Quaternion
	tf::quaternionTFToMsg((((cq - pq)*scale+pq).normalize()), result.orientation);

	//Linearly interpolate the position point
	tf::pointTFToMsg(((cp - pp)*scale+pp), result.position);

}

void aero_odometry::linearInterpolate(const PoseStamped& previous, const PoseStamped& current, const Time& interp_time, PoseStamped& result)
{
	//Extract times
	Time p_time(previous.header.stamp.sec, previous.header.stamp.nsec);
	Time c_time(current.header.stamp.sec,  current.header.stamp.nsec);
	aero_odometry::linearInterpolate(previous.pose, p_time, current.pose, c_time, interp_time, result.pose);
}
}; /*END NAMESPACE aero_odometry */

