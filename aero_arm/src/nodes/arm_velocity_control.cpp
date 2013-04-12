//============================================================================
// Name        : arm_velocity_control.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

/**
 * @file arm_velocity_control.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File
#include <aero_arm/arm_velocity_control.h>

using namespace aero_arm;

Velocity_Controller::Velocity_Controller(ros::NodeHandle nh,
		std::string DesiredPosition, std::string JointVelocity) {
	this->sub_desired_position = nh.subscribe(DesiredPosition, 1,
			&Velocity_Controller::DesiredPositionMSG, this);
	this->pub_joint_velocity = nh.advertise<geometry_msgs::PoseStamped>(
			JointVelocity, 2);
	velocity_timer = nh.createTimer(ros::Duration(0.05),
			&Velocity_Controller::VelocityTimer, this);

	UpdateError();

	PID_X = pid::PIDController(1, 1, 1, pos_err.x_err);
	PID_Y = pid::PIDController(1, 1, 1, pos_err.y_err);
	PID_Z = pid::PIDController(1, 1, 1, pos_err.z_err);
	PID_Roll = pid::PIDController(1, 1, 1, pos_err.roll_err);
	PID_Pitch = pid::PIDController(1, 1, 1, pos_err.pitch_err);
	PID_Yaw = pid::PIDController(1, 1, 1, pos_err.yaw_err);

}

void Velocity_Controller::DesiredPositionMSG(
		const geometry_msgs::PoseStampedConstPtr& object_pose) {
	geometry_msgs::PoseStamped desired_pose_msg;

	tf_listener.waitForTransform("arm_base", object_pose->header.frame_id,
			object_pose->header.stamp, ros::Duration(0.1));
	tf_listener.transformPose("arm_base", *object_pose, desired_pose_msg);

	tf::Stamped<tf::Pose> desired_StampPose;

	tf::poseStampedMsgToTF(desired_pose_msg, desired_StampPose);

	desired_pos.x = desired_StampPose.getOrigin().getX();
	desired_pos.y = desired_StampPose.getOrigin().getY();
	desired_pos.z = desired_StampPose.getOrigin().getZ();

	tf::Matrix3x3 desired_rotation(desired_StampPose.getRotation());

	desired_rotation.getRPY(desired_pos.roll, desired_pos.pitch,
			desired_pos.yaw);

}

void Velocity_Controller::UpdateCurrentPose(void) {
	tf::Stamped<tf::Pose> end_effector_pose;
	tf::Transform end_effector_trans;
	end_effector_trans.setOrigin(tf::Vector3(0, 0, 0));
	end_effector_trans.setRotation(tf::Quaternion(0, 0, 0));
	end_effector_pose.Stamped(end_effector_trans, ros::Time::now(),
			"/jaco_end_effector");

	tf_listener.waitForTransform("/arm_base", end_effector_pose.frame_id_,
			end_effector_pose.stamp_, ros::Duration(0.5));

	tf::Stamped<tf::Pose> current_StampPose;

	tf_listener.transformPose("/arm_base", end_effector_pose,
			current_StampPose);

	current_pos.x = current_StampPose.getOrigin().getX();
	current_pos.y = current_StampPose.getOrigin().getY();
	desired_pos.z = current_StampPose.getOrigin().getZ();

	tf::Matrix3x3 current_rotation(current_StampPose.getRotation());

	current_rotation.getRPY(current_pos.roll, current_pos.pitch,
			current_pos.yaw);

}

void Velocity_Controller::UpdateError(void) {

	UpdateCurrentPose();

	tf::Vector3 desired_origin;
	tf::Quaternion desired_retation;

	tf::Vector3 desired_origin;
	tf::Quaternion desired_retation;

	pos_err.x_err = current_pos.x - desired_pos.x;
	pos_err.y_err = current_pos.y - desired_pos.y;
	pos_err.z_err = current_pos.z - desired_pos.z;
	pos_err.roll_err = current_pos.roll - desired_pos.roll;
	pos_err.pitch_err = current_pos.pitch - desired_pos.pitch;
	pos_err.yaw_err = current_pos.yaw - desired_pos.yaw;

}
void Velocity_Controller::VelocityTimer(const ros::TimerEvent&) {
	UpdateError();

	PID_X.PIDUpdate(pos_err.x_err);
	PID_Y.PIDUpdate(pos_err.y_err);
	PID_Z.PIDUpdate(pos_err.z_err);
	PID_Roll.PIDUpdate(pos_err.roll_err);
	PID_Pitch.PIDUpdate(pos_err.pitch_err);
	PID_Yaw.PIDUpdate(pos_err.yaw_err);

}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "arm_velocity_control");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string DesiredPosition("DesiredPosition"); ///String containing the topic name for goal position
	std::string JointVelocity("JointVelocity"); ///String containing the topic name for JointVelocity

	if (argc < 1) {
		ROS_INFO(
				"Usage: arm_velocity_control desired_position_topic joint_velocity_topic");
		return 1;
	} else {
		//Grab the topic parameters, print warnings if using default values
		if (!param_nh.getParam(DesiredPosition, DesiredPosition))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Desired Position Topic <%s>!", DesiredPosition.c_str(), DesiredPosition.c_str());
		if (!param_nh.getParam(JointVelocity, JointVelocity))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Joint Velocity Topic <%s>!", JointVelocity.c_str(), JointVelocity.c_str());
	}

//Print out received topics
	ROS_DEBUG("Got Desired Position Topic Name: <%s>", DesiredPosition.c_str());
	ROS_DEBUG("Got Joint Velocity Topic Name: <%s>", JointVelocity.c_str());

	ROS_INFO("Starting Up Arm Velocity Controller...");

//create the arm object
	Velocity_Controller arm_velocity(nh, DesiredPosition, JointVelocity);

	ros::spin();
}

