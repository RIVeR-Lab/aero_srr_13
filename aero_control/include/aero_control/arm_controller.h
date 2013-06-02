/*
 * arm_controller.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef ARM_CONTROLLER_H_
#define ARM_CONTROLLER_H_

/* Define to debug without arm */
//#define DEBUG_WITHOUT_ARM
//#define PRINT_DEBUG_INFO
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <jaco_driver/finger_position.h>
#include <aero_srr_msgs/AeroState.h>
#include <aero_srr_msgs/StateTransitionRequest.h>
#include <aero_control/arm_state.h>

namespace aero_control {
#define MAX_PATH_STEPS 20

class ArmController {
public:
	ArmController(ros::NodeHandle nh, ros::NodeHandle param_nh);

private:
	void PathTimerCallback(const ros::TimerEvent&);
	void ObjectPositionMSG(const aero_srr_msgs::ObjectLocationMsgConstPtr& object);
	void ArmStateMSG(const aero_control::arm_stateConstPtr& arm_state);
	void AeroStateMSG(const aero_srr_msgs::AeroStateConstPtr& aero_state);
	typedef struct {
		/* Finger motion */
		bool finger_motion;
		double finger_1_pos;
		double finger_2_pos;
		double finger_3_pos;

		/* Arm Motion */
		bool arm_motion;
		double x_pos;
		double y_pos;
		double z_pos;
		double roll_pos;
		double pitch_pos;
		double yaw_pos;
	} path_t;

	path_t arm_path[MAX_PATH_STEPS];
	int path_steps;
	bool path_planed;

	inline bool SetPathStep(bool finger_motion, double finger_1_pos, double finger_2_pos,
			double finger_3_pos, bool arm_motion, double x_pos, double y_pos, double z_pos,
			double roll_pos, double pitch_pos, double yaw_pos) {

		if (this->path_steps < MAX_PATH_STEPS) {
			/* Finger motion */
			this->arm_path[path_steps].finger_motion = finger_motion;
			this->arm_path[path_steps].finger_1_pos = finger_1_pos;
			this->arm_path[path_steps].finger_2_pos = finger_2_pos;
			this->arm_path[path_steps].finger_3_pos = finger_3_pos;

			/* Arm Motion */
			this->arm_path[path_steps].arm_motion = arm_motion;
			this->arm_path[path_steps].x_pos = x_pos;
			this->arm_path[path_steps].y_pos = y_pos;
			this->arm_path[path_steps].z_pos = z_pos;
			this->arm_path[path_steps].roll_pos = roll_pos;
			this->arm_path[path_steps].pitch_pos = pitch_pos;
			this->arm_path[path_steps].yaw_pos = yaw_pos;
			this->path_steps++;
			return true;
		} else {
			ROS_WARN("Too many path points, you need to increase the maximum number of path points!");
			return false;

		}

	}

	inline void PlanHorizontalPath(void) {
		path_steps = 0;

	//	SetPathStep(true,30,30,30,false,0,0,0,0,0,0);
		SetPathStep(false,0,0,0,true,0.4,-0.1,0.4,-1.421,0.4668,3);
		SetPathStep(false,0,0,0,true,0.4,-0.1,0.6,-1.421,0.4668,3);
		SetPathStep(false,0,0,0,true,0.4,0,0.6,-1.421,0.4668,3);
		SetPathStep(false,0,0,0,true,0.5,0,0.4,-1.421,0.4668,3);
		SetPathStep(false,0,0,0,true,0.3,-0.1,0.4,-1.421,0.4668,3);


		//SetPathStep(true,0,0,0,false,0,0,0,0,0,0);
	}

	ros::NodeHandle nh_;
	ros::Subscriber object_position_sub;
	ros::Subscriber aero_state_sub;
	ros::Subscriber arm_state_sub;

	ros::Publisher arm_position_pub;
	ros::Publisher set_finger_position_pub;

	ros::ServiceClient aero_state_transition_srv_client;

	tf::TransformListener listener;

	geometry_msgs::PoseStamped obj_pose;

	ros::Timer path_timer;
ros::Time path_step_start_time;
	bool active_state;
	uint8_t previous_state;

	bool path_active;
	bool arm_moving;
	bool arm_goal_reached;
	bool path_step_start;

	uint8_t object_type;
	uint8_t path_step_num;

};

}

#endif /* ARM_CONTROLLER_H_ */
