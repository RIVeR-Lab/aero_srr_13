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



void aero_odometry::linearInterpolate(const Pose& previous, const Time& previous_time, const Pose& current, const Time& current_time, const Time& interp_time, Pose& result)
{
	//extract the position/rotation data
	Quaternion pq;
	Quaternion cq;
	Point      pp;
	Point      cp;
	tf::quaternionMsgToTF(previous.orientation, pq);
	tf::quaternionMsgToTF(current.orientation, cq);
	tf::pointMsgToTF(previous.position, pp);
	tf::pointMsgToTF(current.position, cp);
	//Calculate the %change in time
	double scale        = (interp_time-previous_time).toSec()/(current_time-previous_time).toSec();

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

void aero_odometry::linearInterpolate(const Twist& previous, const Time& previous_time, const Twist& current, const Time& current_time, const Time& interp_time, Twist& result){
	//extract the linear and angular velocities
	Vector3 pl;
	Vector3 cl;
	Vector3 pa;
	Vector3 ca;
	tf::vector3MsgToTF(previous.linear, pl);
	tf::vector3MsgToTF(current.linear, cl);
	tf::vector3MsgToTF(previous.angular, pa);
	tf::vector3MsgToTF(current.angular, ca);
	//Calculate the %change in time
	double scale = (interp_time-previous_time).toSec()/(current_time-previous_time).toSec();

	//Linearly interpolate the linear velocity
	tf::vector3TFToMsg((cl-pl)*scale+pl, result.linear);

	//Linear interpolate the angular velocity
	tf::vector3TFToMsg((ca-pa)*scale+pa, result.angular);
}

void aero_odometry::linearInterpolate(const TwistStamped& previous, const TwistStamped& current, const Time& interp_time, TwistStamped& result){
	//Extract times
	Time p_time(previous.header.stamp.sec, previous.header.stamp.nsec);
	Time c_time(current.header.stamp.sec,  current.header.stamp.nsec);
	aero_odometry::linearInterpolate(previous.twist, p_time, current.twist, c_time, interp_time, result.twist);
}

