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
 * @author parallels
 * @brief \todo
 */

//License File
#include <aero_control/arm_position_control.h>

using namespace aero_control;

ArmPositionController::ArmPositionController(ros::NodeHandle nh, ros::NodeHandle param_nh) {

	std::string DesiredPosition("DesiredARMPosition"); ///String containing the topic name for goal position
	std::string ArmError("ArmError"); ///String containing the topic name for arm error
	std::string JointVelocity("CartesianVelocity"); ///String containing the topic name for JointVelocity
	std::string JointAngles("JointAngles"); ///String containing the topic name for JointAngles
	std::string CurrentPosition("ToolPosition"); ///String containing the topic name for CurrentPosition

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(DesiredPosition, DesiredPosition))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Desired Position Topic <%s>!", DesiredPosition.c_str(), DesiredPosition.c_str());
	if (!param_nh.getParam(JointVelocity, JointVelocity))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Joint Velocity Topic <%s>!", JointVelocity.c_str(), JointVelocity.c_str());
	if (!param_nh.getParam(JointAngles, JointAngles))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Joint Angles Topic <%s>!", JointAngles.c_str(), JointAngles.c_str());
	if (!param_nh.getParam(CurrentPosition, CurrentPosition))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Current Position Topic <%s>!", CurrentPosition.c_str(), CurrentPosition.c_str());
	if (!param_nh.getParam(ArmError, ArmError))
		ROS_WARN(
				"Parameter <%s> Not Set. Using Default Arm Error Topic <%s>!", ArmError.c_str(), ArmError.c_str());

//Print out received topics
	ROS_DEBUG("Got Desired Position Topic Name: <%s>", DesiredPosition.c_str());
	ROS_DEBUG("Got Joint Velocity Topic Name: <%s>", JointVelocity.c_str());
	ROS_DEBUG("Got Joint Angles Topic Name: <%s>", JointAngles.c_str());
	ROS_DEBUG("Got Current Position Topic Name: <%s>", CurrentPosition.c_str());
	ROS_DEBUG("Using Arm Error Topic Name: <%s>", ArmError.c_str());

	ROS_INFO("Starting Up Arm Velocity Controller...");

	this->desired_position_sub = nh.subscribe(DesiredPosition, 1,
			&ArmPositionController::DesiredPositionMSG, this);
	this->current_position_sub = nh.subscribe(CurrentPosition, 1,
			&ArmPositionController::CurrentPositionMSG, this);

	this->joint_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(JointVelocity, 2);

	this->joint_angles_sub = nh.subscribe(JointAngles, 1, &ArmPositionController::JointAnglesMSG,
			this);
	this->joint_angles_sub = nh.subscribe(JointAngles, 1, &ArmPositionController::JointAnglesMSG,
			this);
	last_position_time = ros::Time().now();
	running = false;
	//UpdateError();

	PID_X = new pid::PIDController(3, 0, 0.5, pos_err.x_err);
	PID_Y = new pid::PIDController(3, 0, 0.5, pos_err.y_err);
	PID_Z = new pid::PIDController(3, 0, 0.5, pos_err.z_err);
	PID_Roll = new pid::PIDController(3, 0, 0.5, pos_err.roll_err);
	PID_Pitch = new pid::PIDController(3, 0, 0.5, pos_err.pitch_err);
	PID_Yaw = new pid::PIDController(3, 0, 0.5, pos_err.yaw_err);

	linear_gain = 2;
	rotational_gain = 1;

}

ArmPositionController::~ArmPositionController() {
	delete PID_X;
	delete PID_Y;
	delete PID_Z;
	delete PID_Roll;
	delete PID_Pitch;
	delete PID_Yaw;

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
//	ROS_INFO("Desired");
//	ROS_INFO("X_Desired = %f", desired_pos.x);
//	ROS_INFO("Y_Desired = %f", desired_pos.y);
//	ROS_INFO("Z_Desired = %f", desired_pos.z);
//	ROS_INFO("RX_Desired = %f", desired_pos.roll);
//	ROS_INFO("RY_Desired = %f", desired_pos.pitch);
//	ROS_INFO("RZ_Desired = %f", desired_pos.y);

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
//
//	ROS_INFO("X_CUR = %f, X_DES = %f, X_ERR_ = %f", current_pos.x, desired_pos.x, pos_err.x_err);
//	ROS_INFO("Y_CUR = %f, Y_DES = %f, Y_ERR_ = %f", current_pos.y, desired_pos.y, pos_err.y_err);
//	ROS_INFO("Z_CUR = %f, Z_DES = %f, Z_ERR_ = %f", current_pos.z, desired_pos.z, pos_err.z_err);
//	ROS_INFO("rX_CUR = %f, rX_DES = %f, rX_ERR_ = %f", current_pos.roll, desired_pos.roll, pos_err.roll_err);
//	ROS_INFO("rY_CUR = %f, rY_DES = %f, rY_ERR_ = %f", current_pos.pitch, desired_pos.pitch, pos_err.pitch_err);
//	ROS_INFO("rZ_CUR = %f, rZ_DES = %f, rZ_ERR_ = %f", current_pos.yaw, desired_pos.yaw, pos_err.yaw_err);

	if (running == true) {
		UpdatePID();
	}

	if (running == true && (ros::Time().now().toSec() - last_position_time.toSec() > 1)) {
		running = false;
	}
}
void ArmPositionController::UpdatePID(void) {
	Eigen::VectorXf cartisian_velocity(6);

	cartisian_velocity(0) = PID_X->PIDUpdate(pos_err.x_err);
	cartisian_velocity(0) *= linear_gain;

	if (cartisian_velocity(0) > 0.037) {
		cartisian_velocity(0) = 0.037;
	} else if (cartisian_velocity(0) < -0.037) {
		cartisian_velocity(0) = -0.037;
	}
	cartisian_velocity(1) = PID_Y->PIDUpdate(pos_err.y_err);
	cartisian_velocity(1) *= linear_gain;

	if (cartisian_velocity(1) > 0.037) {
		cartisian_velocity(1) = 0.037;
	} else if (cartisian_velocity(1) < -0.037) {
		cartisian_velocity(1) = -0.037;
	}
	cartisian_velocity(2) = PID_Z->PIDUpdate(pos_err.z_err);
	cartisian_velocity(2) *= linear_gain;

	if (cartisian_velocity(2) > 0.037) {
		cartisian_velocity(2) = 0.037;
	} else if (cartisian_velocity(2) < -0.037) {
		cartisian_velocity(2) = -0.037;
	}
	cartisian_velocity(3) = PID_Roll->PIDUpdate(pos_err.roll_err);
	cartisian_velocity(3) *= rotational_gain;

	if (cartisian_velocity(3) > 0.15) {
		cartisian_velocity(3) = 0.15;
	} else if (cartisian_velocity(3) < -0.15) {
		cartisian_velocity(3) = -0.15;
	}
	cartisian_velocity(4) = PID_Pitch->PIDUpdate(pos_err.pitch_err);
	cartisian_velocity(4) *= rotational_gain;

	if (cartisian_velocity(4) > 0.15) {
		cartisian_velocity(4) = 0.15;
	} else if (cartisian_velocity(4) < -0.15) {
		cartisian_velocity(4) = -0.15;
	}
	cartisian_velocity(5) = PID_Yaw->PIDUpdate(pos_err.yaw_err);
	cartisian_velocity(5) *= rotational_gain;

	if (cartisian_velocity(5) > 0.15) {
		cartisian_velocity(5) = 0.15;
	} else if (cartisian_velocity(5) < -0.15) {
		cartisian_velocity(5) = -0.15;
	}

//	ROS_INFO("X_V = %f", cartisian_velocity(0));
//	ROS_INFO("Y_V = %f", cartisian_velocity(1));
//	ROS_INFO("Z_V = %f", cartisian_velocity(2));
//	ROS_INFO("Roll_V = %f", cartisian_velocity(3));
//	ROS_INFO("Pitch_V = %f", cartisian_velocity(4));
//	ROS_INFO("Yaw_V = %f", cartisian_velocity(5));

	geometry_msgs::TwistStamped cartesian_velocity_msg;
	cartesian_velocity_msg.header.frame_id = "/arm_base";
	cartesian_velocity_msg.header.stamp = ros::Time::now();
	cartesian_velocity_msg.twist.linear.x = cartisian_velocity(0);
	cartesian_velocity_msg.twist.linear.y = cartisian_velocity(1);
	cartesian_velocity_msg.twist.linear.z = cartisian_velocity(2);
	cartesian_velocity_msg.twist.angular.x = cartisian_velocity(3);
	cartesian_velocity_msg.twist.angular.y = cartisian_velocity(4);
	cartesian_velocity_msg.twist.angular.z = cartisian_velocity(5);

//	cartesian_velocity_msg.Velocity_X = 100;//cartisian_velocity(0);
//	cartesian_velocity_msg.twist.linear.y = 0;//cartisian_velocity(1);
	//cartesian_velocity_msg.twist.linear.z = 0;//cartisian_velocity(2);
	//cartesian_velocity_msg.twist.angular.x = 0;//cartisian_velocity(3);
	//cartesian_velocity_msg.twist.angular.y = 0;//cartisian_velocity(4);
	//cartesian_velocity_msg.twist.angular.z = 0;//cartisian_velocity(5);

	joint_velocity_pub.publish(cartesian_velocity_msg);

}
void ArmPositionController::JointAnglesMSG(const jaco_driver::joint_anglesConstPtr& joint_angles) {
//	UpdateError();
//
//
//
//
///*
//	ros::Time start = ros::Time().now();
//
//	//Precalculated Jacobian
//	Eigen::MatrixXf jacobian(6, 6);
//
//	jacobian(0, 0) = -(.6590121487
//			* (-sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) - sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3))
//			* sin((double)joint_angles->Angle_J4) + .6590121487 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4)) * cos((double)joint_angles->Angle_J5)
//			- (1.148952620
//					* (-sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//							- sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					- 1.148952620 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4)) * sin((double)joint_angles->Angle_J5)
//			- (-.9411668873 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					+ .9411668873 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			+ (-1.143254838 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//					- 1.143254838 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			+ 1.143254838 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4) + .3810546647 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			- .3810546647 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + 0.113e-1 * cos((double)joint_angles->Angle_J1)
//			- .41 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2);
//
//	jacobian(0, 1) = -(-6590121487 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			+ .6590121487 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4) * cos((double)joint_angles->Angle_J5)
//			- (-1.148952620 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					+ 1.148952620 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4) * sin((double)joint_angles->Angle_J5)
//			- (.9411668873 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//					+ .9411668873 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			+ (-1.143254838 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					+ 1.143254838 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .3810546647 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			- .3810546647 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .41 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2);
//
//	jacobian(0, 2) = -(.6590121487 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			- .6590121487 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4) * cos((double)joint_angles->Angle_J5)
//			- (1.148952620 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					- 1.148952620 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4) * sin((double)joint_angles->Angle_J5)
//			- (-.9411668873 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//					- .9411668873 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			+ (1.143254838 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					- 1.143254838 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			+ .3810546647 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//			+ .3810546647 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3);
//
//	jacobian(0, 3) =
//			-(.6590121487
//					* (cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3))
//					* cos((double)joint_angles->Angle_J4) - .6590121487 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4)) * cos((double)joint_angles->Angle_J5)
//					- (-1.148952620
//							* (cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//									+ cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//							- 1.148952620 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4)) * sin((double)joint_angles->Angle_J5)
//					+ (1.143254838 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//							+ 1.143254838 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					- 1.143254838 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4);
//
//	jacobian(0, 4) =
//			(.6590121487
//					* (cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3))
//					* sin((double)joint_angles->Angle_J4) + .6590121487 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4)) * sin((double)joint_angles->Angle_J5)
//					- (1.148952620
//							* (cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//									+ cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//							- 1.148952620 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4)) * cos((double)joint_angles->Angle_J5)
//					+ (.9411668873 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//							- .9411668873 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J5);
//
//	jacobian(0, 5) = 0;
//
//	jacobian(1, 0) = -(.6590121487
//			* (-cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) - cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3))
//			* sin((double)joint_angles->Angle_J4) - .6590121487 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4)) * cos((double)joint_angles->Angle_J5)
//			- (1.148952620
//					* (-cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//							- cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					+ 1.148952620 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4)) * sin((double)joint_angles->Angle_J5)
//			- (-.9411668873 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					+ .9411668873 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			+ (-1.143254838 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//					- 1.143254838 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- 1.143254838 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4) + .3810546647 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			- .3810546647 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) - 0.113e-1 * sin((double)joint_angles->Angle_J1)
//			- .41 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2);
//
//	jacobian(1, 1) = -(.6590121487 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			- .6590121487 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4) * cos((double)joint_angles->Angle_J5)
//			- (1.148952620 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					- 1.148952620 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4) * sin((double)joint_angles->Angle_J5)
//			- (-.9411668873 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//					- .9411668873 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			+ (1.143254838 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					- 1.143254838 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			+ .3810546647 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			+ .3810546647 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) - .41 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2);
//
//	jacobian(1, 2) = -(-.6590121487 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			+ .6590121487 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4) * cos((double)joint_angles->Angle_J5)
//			- (-1.148952620 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					+ 1.148952620 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4) * sin((double)joint_angles->Angle_J5)
//			- (.9411668873 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//					+ .9411668873 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			+ (-1.143254838 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					+ 1.143254838 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .3810546647 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//			- .3810546647 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3);
//
//	jacobian(1, 3) = -(.6590121487
//			* (-sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) - sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3))
//			* cos((double)joint_angles->Angle_J4) - .6590121487 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4)) * cos((double)joint_angles->Angle_J5)
//			- (-1.148952620
//					* (-sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//							- sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//					- 1.148952620 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4)) * sin((double)joint_angles->Angle_J5)
//			+ (-1.143254838 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//					- 1.143254838 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//			- 1.143254838 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4);
//
//	jacobian(1, 4) = (.6590121487
//			* (-sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) - sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3))
//			* sin((double)joint_angles->Angle_J4) + .6590121487 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4)) * sin((double)joint_angles->Angle_J5)
//			- (1.148952620
//					* (-sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//							- sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					- 1.148952620 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4)) * cos((double)joint_angles->Angle_J5)
//			+ (-.9411668873 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					+ .9411668873 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J5);
//
//	jacobian(1, 5) = 0;
//
//	jacobian(2, 0) = 0;
//
//	jacobian(2, 1) = -(.6590121487 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) + .6590121487 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			* cos((double)joint_angles->Angle_J5)
//			- (1.148952620 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) + 1.148952620 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					* sin((double)joint_angles->Angle_J5)
//			- (-.9411668873 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .9411668873 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			+ (1.143254838 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) + 1.143254838 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .3810546647 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) + .3810546647 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .41 * sin((double)joint_angles->Angle_J2);
//
//	jacobian(2, 2) = -(-.6590121487 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - .6590121487 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			* cos((double)joint_angles->Angle_J5)
//			- (-1.148952620 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - 1.148952620 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					* sin((double)joint_angles->Angle_J5)
//			- (.9411668873 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) - .9411668873 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			+ (-1.143254838 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - 1.143254838 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .3810546647 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .3810546647 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3);
//
//	jacobian(2, 3) = -(-.6590121487 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .6590121487 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//			* cos((double)joint_angles->Angle_J5)
//			+ (-1.148952620 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + 1.148952620 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//					* sin((double)joint_angles->Angle_J5)
//			+ (-1.143254838 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + 1.143254838 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4);
//
//	jacobian(2, 4) = (-.6590121487 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .6590121487 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			* sin((double)joint_angles->Angle_J5)
//			- (-1.148952620 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + 1.148952620 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					* cos((double)joint_angles->Angle_J5)
//			+ (-.9411668873 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - .9411668873 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J5);
//
//	jacobian(2, 5) = 0;
//
//	jacobian(3, 0) = 0;
//
//	jacobian(3, 1) = -sin((double)joint_angles->Angle_J1);
//
//	jacobian(3, 2) = sin((double)joint_angles->Angle_J1);
//
//	jacobian(3, 3) = cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//	jacobian(3, 4) = -(.8191520445 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//			+ .8191520445 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .8191520445 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4) + .5735764360 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			- .5735764360 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//	jacobian(3, 5) =
//			(.4698463102
//					* (cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3))
//					* sin((double)joint_angles->Angle_J4) + .4698463102 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4)) * cos((double)joint_angles->Angle_J5)
//					+ (.8191520445
//							* (cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//									+ cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//							- .8191520445 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4)) * sin((double)joint_angles->Angle_J5)
//					+ (.6710100720 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//							- .6710100720 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//					- (.4698463102 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//							+ .4698463102 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//					- .4698463102 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4) + .3289899279 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					- .3289899279 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//	jacobian(4, 0) = 0;
//
//	jacobian(4, 1) = -cos((double)joint_angles->Angle_J1);
//
//	jacobian(4, 2) = cos((double)joint_angles->Angle_J1);
//
//	jacobian(4, 3) = -sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) + sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//	jacobian(4, 4) = -(-.8191520445 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//			- .8191520445 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .8191520445 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4) - .5735764360 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			+ .5735764360 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//	jacobian(4, 5) = (.4698463102
//			* (-sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) - sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3))
//			* sin((double)joint_angles->Angle_J4) + .4698463102 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4)) * cos((double)joint_angles->Angle_J5)
//			+ (.8191520445
//					* (-sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//							- sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					- .8191520445 * cos((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J4)) * sin((double)joint_angles->Angle_J5)
//			+ (-.6710100720 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//					+ .6710100720 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			- (-.4698463102 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)
//					- .4698463102 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .4698463102 * cos((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J4) - .3289899279 * sin((double)joint_angles->Angle_J1) * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)
//			+ .3289899279 * sin((double)joint_angles->Angle_J1) * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//	jacobian(5, 0) = -1;
//
//	jacobian(5, 1) = 0;
//
//	jacobian(5, 2) = 0;
//
//	jacobian(5, 3) = -cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//	jacobian(5, 4) = -(-.8191520445 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .8191520445 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .5735764360 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - .5735764360 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//	jacobian(5, 5) = (-.4698463102 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .4698463102 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			* cos((double)joint_angles->Angle_J5)
//			+ (-.8191520445 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .8191520445 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J4)
//					* sin((double)joint_angles->Angle_J5)
//			+ (-.6710100720 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - .6710100720 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3)) * cos((double)joint_angles->Angle_J5)
//			- (-.4698463102 * cos((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3) + .4698463102 * sin((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3)) * sin((double)joint_angles->Angle_J4)
//			- .3289899279 * cos((double)joint_angles->Angle_J2) * cos((double)joint_angles->Angle_J3) - .3289899279 * sin((double)joint_angles->Angle_J2) * sin((double)joint_angles->Angle_J3);
//
//
//	Eigen::VectorXf joint_velocity(6);
//
//
//	Eigen::MatrixXf pinvtemp(6, 6);
//
//
//	pinvtemp=(jacobian*jacobian.transpose());
//
//	joint_velocity = (jacobian.transpose()*pinvtemp.inverse())* cartisian_velocity;
//	ROS_INFO("Time1= %f",(start - ros::Time().now()).sec);
//
//	jaco_driver::joint_velocity joint_velocity_msg;
//
//	joint_velocity_msg.Velocity_J1 = joint_velocity(0);
//	joint_velocity_msg.Velocity_J2 = joint_velocity(1);
//	joint_velocity_msg.Velocity_J3 = joint_velocity(2);
//	joint_velocity_msg.Velocity_J4 = joint_velocity(3);
//	joint_velocity_msg.Velocity_J5 = joint_velocity(4);
//	joint_velocity_msg.Velocity_J6 = joint_velocity(5);
//
//	ROS_INFO("J1_V = %f",joint_velocity(0));
//	ROS_INFO("J2_V = %f",joint_velocity(1));
//	ROS_INFO("J3_V = %f",joint_velocity(2));
//	ROS_INFO("J4_V = %f",joint_velocity(3));
//	ROS_INFO("J5_V = %f",joint_velocity(4));
//	ROS_INFO("J6_V = %f",joint_velocity(5));
//
//	joint_velocity_pub.publish(joint_velocity_msg);*/
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

