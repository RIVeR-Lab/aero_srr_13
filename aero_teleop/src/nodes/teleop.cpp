//============================================================================
// Name        : base_servo_control.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

/**
 * @file base_servo_control.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File
#include <aero_teleop/teleop.h>

using namespace aero_teleop;

Teleop::Teleop(ros::NodeHandle nh, std::string Joystick, std::string BaseVelocity,
		int _deadman_button, double _turn_scale, double _drive_scale) {
	this->deadman_button = _deadman_button;
	this->turn_scale = (double) _turn_scale;
	this->drive_scale = (double) _drive_scale;

	base_velocity.linear.x = 0;
	base_velocity.linear.y = 0;
	base_velocity.linear.z = 0;
	base_velocity.angular.x = 0;
	base_velocity.angular.y = 0;
	base_velocity.angular.z = 0;

	last_joy_time = ros::Time().now();

	this->joystick_sub = nh.subscribe(Joystick, 1, &Teleop::JoystickMSG, this);

	this->base_velocity_pub = nh.advertise<geometry_msgs::Twist>(BaseVelocity, 2, true);

	this->send_vel_timer = nh.createTimer(ros::Duration(0.05), &Teleop::SendVelTimerCallback, this);
	send_vel_timer.stop();

}



void Teleop::SendVelTimerCallback(const ros::TimerEvent&) {
	if ((ros::Time().now().toSec() - last_joy_time.toSec()) > 0.2) {
		base_velocity.linear.x = 0;
		base_velocity.angular.z = 0;
	}
	base_velocity_pub.publish(base_velocity);
}

void Teleop::JoystickMSG(const sensor_msgs::JoyConstPtr& joystick) {



	DriveMode();



}
void Teleop::DriveMode(void)
{
	base_velocity.linear.x = joystick->axes[1] * this->drive_scale;
	base_velocity.angular.z = joystick->axes[0] * this->turn_scale;
	last_joy_time = ros::Time().now();
	send_vel_timer.start();

}



int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "teleop");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string Joystick("joy"); ///String containing the topic name for Joystick
	std::string BaseVelocity("BaseVelocity"); ///String containing the topic name for BaseVelocity

	const std::string DeadmanButton("deadman_button"); ///String containing the topic name for DeadmanButton
	int deadman_button;
	const std::string TurnScale("turn_scale"); ///String containing the topic name for TurnScale
	double turn_scale;
	const std::string DriveScale("drive_scale"); ///String containing the topic name for DriveScale
	double drive_scale;

	if (argc < 1) {
		ROS_INFO( "Usage: teleop desired_position_topic base_velocity_topic");

		return 1;
	} else {
		//Grab the topic parameters, print warnings if using default values
		if (!param_nh.getParam(Joystick, Joystick)) {
			ROS_WARN(
					"Parameter <Joystick> Not Set. Using Default Joystick Topic <%s>!", Joystick.c_str());
		}
		if (!param_nh.getParam(BaseVelocity, BaseVelocity)) {
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Base Velocity Topic <%s>!", BaseVelocity.c_str(), BaseVelocity.c_str());
		}
		param_nh.param(DeadmanButton, deadman_button, 0);

		param_nh.param(TurnScale, turn_scale, 1.0);

		param_nh.param(DriveScale, drive_scale, 1.0);

	}


//Print out received topics
ROS_DEBUG("Joystick Topic Name: <%s>", Joystick.c_str());
ROS_DEBUG("Base Velocity Topic Name: <%s>", BaseVelocity.c_str());
ROS_DEBUG("Deadman Button: <%d>", deadman_button);
ROS_DEBUG("Turn Scale: <%f>", turn_scale);
ROS_DEBUG("Drive Scale: <%f>", drive_scale);

ROS_INFO("Starting Up Aero Teleop...");

//create the arm object
Teleop teleop(nh, Joystick, BaseVelocity, deadman_button, turn_scale, drive_scale);

ros::spin();
}

