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
#include <dynamic_reconfigure/server.h>
#include <aero_control/BaseServoPIDConfig.h>
#include <aero_srr_msgs/AeroState.h>
#include <aero_srr_msgs/StateTransitionRequest.h>

namespace aero_control {

class BaseServoController {
public:
	BaseServoController(ros::NodeHandle nh, ros::NodeHandle param_nh);
	~BaseServoController();

private:

	typedef struct {
		double x_err;
		double y_err;
	} base_position_error;
	void BaseServoStart(void);
	void BaseServoStop(void);
	void PIDConfigCallback(aero_control::BaseServoPIDConfig &config, uint32_t level);
	void ErrorUpdateTimerCallback(const ros::TimerEvent&);
	void StateTimeoutTimerCallback(const ros::TimerEvent&);
	void DesiredPositionMSG(const geometry_msgs::PoseStampedConstPtr& object_pose);
	void AeroStateMSG(const aero_srr_msgs::AeroState& aero_state);
	void UpdatePID(void);

	void UpdateError(void);

	inline double MaxLinearVel(void) {
		return 0.2;
	}

	inline double MaxAngularVel(void) {
		return 0.5;
	}

	inline double ErrorRange(void) {
		return 0.05;
	}

	ros::NodeHandle nh_;
	ros::Subscriber desired_position_sub;
	ros::Publisher base_velocity_pub;
	ros::Publisher workspace_postion_pub;
	ros::Subscriber aero_state_sub;
	ros::ServiceClient aero_state_transition_srv_client;
	tf::TransformListener tf_listener;

	geometry_msgs::PoseStamped desired_pose;
	geometry_msgs::PoseStamped workspace_pose;

	dynamic_reconfigure::Server<aero_control::BaseServoPIDConfig> dr_server;
	dynamic_reconfigure::Server<aero_control::BaseServoPIDConfig>::CallbackType dr_call;

	base_position_error pos_err;

	pid::PIDController *PID_X;
	pid::PIDController *PID_Y;

	ros::Timer error_update_timer;
	ros::Timer state_timeout_timer;


	bool error_update_timer_flag;

	ros::Time last_position_time;
	ros::Time in_range_time;
	ros::Time PID_start_time;

	float linear_gain;
	float rotational_gain;

	bool active_state;


	uint8_t previous_state;

};

}

#endif /* BASE_SERVO_CONTROL_H_ */
