/*
 * base_servo_control.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef BASE_SERVO_CONTROL_H_
#define BASE_SERVO_CONTROL_H_




#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <string.h>
#include <aero_control/pid_control.h>




namespace aero_control {



class BaseServoController {
public:
	BaseServoController(ros::NodeHandle nh, std::string DesiredPosition,std::string BaseVelocity);
	~BaseServoController();
private:


	typedef struct {
		double x_err;
		double y_err;
	}base_position_error;


	void ErrorUpdateTimerCallback(const ros::TimerEvent&);
	void DesiredPositionMSG(const geometry_msgs::PoseStampedConstPtr& object_pose);
	void UpdatePID(void);

	void UpdateError(void);

	inline double MaxLinearVel(void)
	{
		return 1.0;
	}

	inline double MaxAngularVel(void)
	{
		return 1.0;
	}


	ros::NodeHandle nh_;
	ros::Subscriber desired_position_sub;
	ros::Publisher base_velocity_pub;

	tf::TransformListener tf_listener;

	geometry_msgs::PoseStamped desired_pose;



	base_position_error pos_err;

	pid::PIDController *PID_X;
	pid::PIDController *PID_Y;

	ros::Timer error_update_timer;
	bool error_update_timer_flag;

	ros::Time last_position_time;

	float linear_gain;
	float rotational_gain;
};

}

#endif /* BASE_SERVO_CONTROL_H_ */
