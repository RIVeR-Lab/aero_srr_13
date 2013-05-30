/*
 * teleop.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef TELEOP_H_
#define TELEOP_H_




#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <string.h>
#include <stdlib.h>


namespace aero_teleop {



class Teleop {
public:
	Teleop(ros::NodeHandle nh, std::string Joystick,
			std::string BaseVelocity,int _deadman_button,double _turn_scale,double _drive_scale);
private:
	void JoystickMSG(	const sensor_msgs::JoyConstPtr& joystick) ;
	void SendVelTimerCallback(const ros::TimerEvent&);
	void SupervisorTimerCallback(const ros::TimerEvent&);
	ros::Timer error_update_timer;
	int deadman_button;
	double turn_scale;
	double drive_scale;

	geometry_msgs::Twist base_velocity;

	ros::Subscriber joystick_sub;
	ros::Publisher base_velocity_pub;

	ros::Timer supervisor_cmd_timer;
	ros::Timer send_vel_timer;
	ros::Time last_joy_time;

};

}

#endif /* TELEOP_H_ */
