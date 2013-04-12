/*
 * arm_controller.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef ARM_VELOCITY_CONTROL_H_
#define ARM_VELOCITY_CONTROL_H_




#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <string.h>
#include <aero_arm/pid_control.h>
#include <jaco_driver/joint_velocity.h>



namespace aero_arm {
typedef struct {
	double x_err;
	double y_err;
	double z_err;
	double roll_err;
	double pitch_err;
	double yaw_err;
}position_error;

typedef struct {
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
}position;

class Velocity_Controller {
public:
	Velocity_Controller(ros::NodeHandle nh, std::string DesiredPosition,std::string JointVelocity);

private:
	void DesiredPositionMSG(const geometry_msgs::PoseStampedConstPtr& object_pose);
	void VelocityTimer(const ros::TimerEvent&);
	void UpdateCurrentPose(void);
	void UpdateError(void);
	ros::NodeHandle nh_;
	ros::Subscriber sub_desired_position;
	ros::Publisher pub_joint_velocity;
	ros::Timer velocity_timer;
	tf::TransformListener tf_listener;


	position desired_pos;
	position current_pos;
	position_error pos_err;

	pid::PIDController PID_X;
	pid::PIDController PID_Y;
	pid::PIDController PID_Z;
	pid::PIDController PID_Roll;
	pid::PIDController PID_Pitch;
	pid::PIDController PID_Yaw;


};

}

#endif /* ARM_VELOCITY_CONTROL_H_ */
