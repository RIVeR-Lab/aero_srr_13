/*
 * arm_position_control.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef ARM_POSITION_CONTROL_H_
#define ARM_POSITION_CONTROL_H_




#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <string.h>
#include <aero_control/pid_control.h>
#include <jaco_driver/joint_velocity.h>

#include <jaco_driver/joint_angles.h>
#include <Eigen/Dense>



namespace aero_control {


class ArmPositionController {
public:
	ArmPositionController(ros::NodeHandle nh, std::string DesiredPosition,std::string JointVelocity, std::string JointAngles, std::string CurrentPosition);
	~ArmPositionController();
private:

	typedef struct {
		double x_err;
		double y_err;
		double z_err;
		double roll_err;
		double pitch_err;
		double yaw_err;
	}arm_position_error;

	typedef struct {
		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
	}arm_position;

		void CurrentPositionMSG(const geometry_msgs::PoseStampedConstPtr& current_pose);
		void DesiredPositionMSG(const geometry_msgs::PoseStampedConstPtr& object_pose);
	void JointAnglesMSG(const jaco_driver::joint_anglesConstPtr& joint_angles);
	void UpdateCurrentPose(void);
	void UpdatePID(void);

	void UpdateError(void);
	ros::NodeHandle nh_;
	ros::Subscriber desired_position_sub;
	ros::Subscriber current_position_sub;
	ros::Publisher joint_velocity_pub;
	ros::Subscriber joint_angles_sub;

	tf::TransformListener tf_listener;



	arm_position desired_pos;
	arm_position current_pos;
	arm_position_error pos_err;

	pid::PIDController *PID_X;
	pid::PIDController *PID_Y;
	pid::PIDController *PID_Z;
	pid::PIDController *PID_Roll;
	pid::PIDController *PID_Pitch;
	pid::PIDController *PID_Yaw;

	ros::Time last_position_time;

	bool running;
	float linear_gain;
	float rotational_gain;
};

}

#endif /* ARM_POSITION_CONTROL_H_ */
