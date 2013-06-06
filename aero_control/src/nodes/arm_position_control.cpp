//============================================================================
// Name        : arm_position_control.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

/**
 * @file arm_position_control.cpp
 *
 * @date   Feb 20, 2013
 * @author mdedonato
 * @brief \todo
 */

//License File
#include <aero_control/arm_position_control.h>

using namespace aero_control;

ArmPositionController::ArmPositionController(ros::NodeHandle nh, ros::NodeHandle param_nh) {

	std::string desired_arm_position("desired_arm_position"); ///String containing the topic name for goal position
	std::string arm_state("arm_state"); ///String containing the topic name for arm state
	std::string cartesian_velocity("cartesian_velocity"); ///String containing the topic name for cartesian_velocity
	std::string current_position("current_position"); ///String containing the topic name for current_position

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(desired_arm_position, desired_arm_position))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Desired Position Topic <%s>!", desired_arm_position.c_str(), desired_arm_position.c_str());
	if (!param_nh.getParam(cartesian_velocity, cartesian_velocity))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Cartesian Velocity Topic <%s>!", cartesian_velocity.c_str(), cartesian_velocity.c_str());
	if (!param_nh.getParam(current_position, current_position))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Current Position Topic <%s>!", current_position.c_str(), current_position.c_str());
	if (!param_nh.getParam(arm_state, arm_state))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Arm State Topic <%s>!", arm_state.c_str(), arm_state.c_str());

//Print out received topics
	ROS_DEBUG("Got Desired Position Topic Name: <%s>", desired_arm_position.c_str());
	ROS_DEBUG("Got Cartesian Velocity Topic Name: <%s>", cartesian_velocity.c_str());
	ROS_DEBUG("Got Current Position Topic Name: <%s>", current_position.c_str());
	ROS_DEBUG("Using Arm State Topic Name: <%s>", arm_state.c_str());

	ROS_INFO("Starting Up Arm Velocity Controller...");

	this->desired_position_sub = nh.subscribe(desired_arm_position, 1,
			&ArmPositionController::DesiredPositionMSG, this);
	this->current_position_sub = nh.subscribe(current_position, 1,
			&ArmPositionController::CurrentPositionMSG, this);

	this->cartesian_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(cartesian_velocity, 2);


	this->arm_state_pub = nh.advertise<aero_control::arm_state>(arm_state, 2, true);

	last_position_time = ros::Time().now();
	goal_time = ros::Time().now();

	this->desired_pos.x=0;
	this->desired_pos.y=0;
	this->desired_pos.z=0;
	this->desired_pos.roll=0;
	this->desired_pos.pitch=0;
	this->desired_pos.yaw=0;


	running = false;
	goal_reached = false;
	last_goal_reached = true;
	PID_X = new pid::PIDController(0.6, 0, 0.45, 0);
	PID_Y = new pid::PIDController(0.6, 0, 0.45, 0);
	PID_Z = new pid::PIDController(0.6, 0, 0.45, 0);
	PID_Roll = new pid::PIDController(0.65, 0, 0.3, 0);
	PID_Pitch = new pid::PIDController(0.65, 0, 0.3, 0);
	PID_Yaw = new pid::PIDController(0.65, 0, 0.3, 0);

	linear_gain = 0.5;
	rotational_gain = 1;




	//	f = boost::bind(&ArmPositionController::callback,this, _1, _2);
		//server.setCallback(f);
}

ArmPositionController::~ArmPositionController() {
	delete PID_X;
	delete PID_Y;
	delete PID_Z;
	delete PID_Roll;
	delete PID_Pitch;
	delete PID_Yaw;

}

void  ArmPositionController::callback(aero_control::AeroArmVelocityConfig &config, uint32_t level) {

	PID_X->SetPID(config.linear_P,config.linear_I,config.linear_D);
	PID_Y->SetPID(config.linear_P,config.linear_I,config.linear_D);
	PID_Z->SetPID(config.linear_P,config.linear_I,config.linear_D);


	PID_Roll->SetPID(config.rotational_P,config.rotational_I_r,config.rotational_D_r);
	PID_Pitch->SetPID(config.rotational_P,config.rotational_I_r,config.rotational_D_r);
	PID_Yaw->SetPID(config.rotational_P,config.rotational_I_r,config.rotational_D_r);

	linear_gain = config.gain;
	rotational_gain = config.gain_r;
}

void ArmPositionController::CurrentPositionMSG(
		const geometry_msgs::PoseStampedConstPtr& current_pose) {
	geometry_msgs::PoseStamped current_pose_api;

	try {
		tf_listener.waitForTransform("/jaco_api_origin", current_pose->header.frame_id,
				current_pose->header.stamp, ros::Duration(1.0));

		tf_listener.transformPose("/jaco_api_origin", *current_pose, current_pose_api);

		tf::Stamped<tf::Pose> Current_StampPose;

		tf::poseStampedMsgToTF(current_pose_api, Current_StampPose);

		current_pos.x = Current_StampPose.getOrigin().getX();
		current_pos.y = Current_StampPose.getOrigin().getY();
		current_pos.z = Current_StampPose.getOrigin().getZ();

		tf::Matrix3x3 current_rotation(Current_StampPose.getRotation());

		current_rotation.getRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
		UpdateError();

	} catch (std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(1, e.what());
	}

}
void ArmPositionController::DesiredPositionMSG(
		const geometry_msgs::PoseStampedConstPtr& object_pose) {
	geometry_msgs::PoseStamped desired_pose_msg;
	try {
		tf_listener.waitForTransform("/jaco_api_origin", object_pose->header.frame_id,
				object_pose->header.stamp, ros::Duration(0.1));
		tf_listener.transformPose("/jaco_api_origin", *object_pose, desired_pose_msg);

		tf::Stamped<tf::Pose> desired_StampPose;

		tf::poseStampedMsgToTF(desired_pose_msg, desired_StampPose);

		desired_pos.x = desired_StampPose.getOrigin().getX();
		desired_pos.y = desired_StampPose.getOrigin().getY();
		desired_pos.z = desired_StampPose.getOrigin().getZ();

		tf::Matrix3x3 desired_rotation(desired_StampPose.getRotation());

		desired_rotation.getRPY(desired_pos.roll, desired_pos.pitch, desired_pos.yaw);

		running = true;
		last_position_time = ros::Time().now();

	} catch (std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(1, e.what());
	}

}

void ArmPositionController::UpdateCurrentPose(void) {
	tf::Stamped<tf::Pose> end_effector_pose;
	end_effector_pose.setOrigin(tf::Vector3(0, 0, 0));
	end_effector_pose.setRotation(tf::Quaternion(0, 0, 0, 1));
	end_effector_pose.frame_id_ = "/jaco_end_effector";
	end_effector_pose.stamp_ = ros::Time::now();
	try {
		tf_listener.waitForTransform("/jaco_api_origin", end_effector_pose.frame_id_,
				end_effector_pose.stamp_, ros::Duration(1.0));

		tf::Stamped<tf::Pose> current_StampPose;

		tf_listener.transformPose("/jaco_api_origin", end_effector_pose, current_StampPose);

		current_pos.x = current_StampPose.getOrigin().getX();
		current_pos.y = current_StampPose.getOrigin().getY();
		current_pos.z = current_StampPose.getOrigin().getZ();

		tf::Matrix3x3 current_rotation(current_StampPose.getRotation());

		current_rotation.getRPY(this->current_pos.roll, current_pos.pitch, current_pos.yaw);

	} catch (std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(1, e.what());
	}

}

void ArmPositionController::UpdateError(void) {

	pos_err.x_err = desired_pos.x - current_pos.x;
	pos_err.y_err = desired_pos.y - current_pos.y;
	pos_err.z_err = desired_pos.z - current_pos.z;
	pos_err.roll_err = desired_pos.roll - current_pos.roll;
	pos_err.pitch_err = desired_pos.pitch - current_pos.pitch;
	pos_err.yaw_err = desired_pos.yaw - current_pos.yaw;


	/* Check if we are at the goal */
	if (pos_err.x_err < LinearErrorRange() && pos_err.y_err < LinearErrorRange()
			&& pos_err.z_err < LinearErrorRange() && pos_err.roll_err < RotationalErrorRange()
			&& pos_err.pitch_err < RotationalErrorRange()
			&& pos_err.yaw_err < RotationalErrorRange()) {
		/* if we have been at the goal for a given time */
		if (ros::Time().now().toSec() - goal_time.toSec() > 1) {
			goal_reached = true;//Goal has been reached
		}
	} else {
		goal_reached = false;//goal not reached

		goal_time = ros::Time().now();//update current time

	}

	SendArmStateMSG();//send the arm state

	if (running == true) {
		UpdatePID();
	}

	if (running == true && (ros::Time().now().toSec() - last_position_time.toSec() > 1)) {
		running = false;
	}
}

void ArmPositionController::SendArmStateMSG(void) {
  if(last_goal_reached!=goal_reached){
	aero_control::arm_state state_msg;

	state_msg.header.stamp = ros::Time().now();

	state_msg.moving = running;
	state_msg.goal_reached = goal_reached;
	state_msg.error.x_error = pos_err.x_err;
	state_msg.error.y_error = pos_err.y_err;
	state_msg.error.z_error = pos_err.z_err;
	state_msg.error.roll_error = pos_err.roll_err;
	state_msg.error.pitch_error = pos_err.pitch_err;
	state_msg.error.yaw_error = pos_err.yaw_err;

	arm_state_pub.publish(state_msg);
	last_goal_reached = goal_reached;
  }
}

void ArmPositionController::UpdatePID(void) {

	geometry_msgs::TwistStamped cartesian_velocity_msg;


	cartesian_velocity_msg.twist.linear.x = PID_X->PIDUpdate(pos_err.x_err);
	cartesian_velocity_msg.twist.linear.x *= linear_gain;

	if (cartesian_velocity_msg.twist.linear.x > MaxLinearVel()) {
		cartesian_velocity_msg.twist.linear.x = MaxLinearVel();
	} else if (cartesian_velocity_msg.twist.linear.x < -MaxLinearVel()) {
		cartesian_velocity_msg.twist.linear.x = -MaxLinearVel();
	}
	cartesian_velocity_msg.twist.linear.y = PID_Y->PIDUpdate(pos_err.y_err);
	cartesian_velocity_msg.twist.linear.y *= linear_gain;

	if (cartesian_velocity_msg.twist.linear.y > MaxLinearVel()) {
		cartesian_velocity_msg.twist.linear.y = MaxLinearVel();
	} else if (cartesian_velocity_msg.twist.linear.y < -MaxLinearVel()) {
		cartesian_velocity_msg.twist.linear.y = -MaxLinearVel();
	}
	cartesian_velocity_msg.twist.linear.z = PID_Z->PIDUpdate(pos_err.z_err);
	cartesian_velocity_msg.twist.linear.z *= linear_gain;

	if (cartesian_velocity_msg.twist.linear.z > MaxLinearVel()) {
		cartesian_velocity_msg.twist.linear.z = MaxLinearVel();
	} else if (cartesian_velocity_msg.twist.linear.z < -MaxLinearVel()) {
		cartesian_velocity_msg.twist.linear.z = -MaxLinearVel();
	}
	cartesian_velocity_msg.twist.angular.x = PID_Roll->PIDUpdate(pos_err.roll_err);
	cartesian_velocity_msg.twist.angular.x *= rotational_gain;

	if (cartesian_velocity_msg.twist.angular.x > MaxAngularVel()) {
		cartesian_velocity_msg.twist.angular.x = MaxAngularVel();
	} else if (cartesian_velocity_msg.twist.angular.x < -MaxAngularVel()) {
		cartesian_velocity_msg.twist.angular.x = -MaxAngularVel();
	}
	cartesian_velocity_msg.twist.angular.y = PID_Pitch->PIDUpdate(pos_err.pitch_err);
	cartesian_velocity_msg.twist.angular.y *= rotational_gain;

	if (cartesian_velocity_msg.twist.angular.y > MaxAngularVel()) {
		cartesian_velocity_msg.twist.angular.y = MaxAngularVel();
	} else if (cartesian_velocity_msg.twist.angular.y < -MaxAngularVel()) {
		cartesian_velocity_msg.twist.angular.y = -MaxAngularVel();
	}
	cartesian_velocity_msg.twist.angular.z = PID_Yaw->PIDUpdate(pos_err.yaw_err);
	cartesian_velocity_msg.twist.angular.z *= rotational_gain;

	if (cartesian_velocity_msg.twist.angular.z > MaxAngularVel()) {
		cartesian_velocity_msg.twist.angular.z = MaxAngularVel();
	} else if (cartesian_velocity_msg.twist.angular.z < -MaxAngularVel()) {
		cartesian_velocity_msg.twist.angular.z = -MaxAngularVel();
	}



	cartesian_velocity_msg.header.frame_id = "/arm_base";
	cartesian_velocity_msg.header.stamp = ros::Time::now();



	cartesian_velocity_pub.publish(cartesian_velocity_msg);

}



int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "arm_position_control");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

//create the arm object
	ArmPositionController arm_position(nh, param_nh);

	ros::spin();
}

