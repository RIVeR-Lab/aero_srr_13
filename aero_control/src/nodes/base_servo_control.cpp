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
#include <aero_control/base_servo_control.h>

using namespace aero_control;

BaseServoController::BaseServoController(ros::NodeHandle nh, std::string DesiredPosition,std::string WorkspacePosition,
		std::string BaseVelocity) {

	this->desired_position_sub = nh.subscribe(DesiredPosition, 1,
			&BaseServoController::DesiredPositionMSG, this);
this->workspace_postion_pub= nh.advertise<geometry_msgs::PointStamped>(WorkspacePosition,1,true);
	this->base_velocity_pub = nh.advertise<geometry_msgs::Twist>(BaseVelocity, 2);

	this->error_update_timer = nh.createTimer(ros::Duration(0.1),
			&BaseServoController::ErrorUpdateTimerCallback, this);


	this->error_update_timer.stop();
	error_update_timer_flag = false;
	last_position_time = ros::Time().now();

	PID_X = new pid::PIDController(0, 0, 0, pos_err.x_err);
	PID_Y = new pid::PIDController(0, 0, 0, pos_err.y_err);

	linear_gain = 2;
	rotational_gain = 1;


	this->workspace_pose.pose.position.x = 0;
	this->workspace_pose.pose.position.y = 0;
	this->workspace_pose.pose.position.z = 0;
	tf::Quaternion q;

	q.setRPY(0, 0, 0);

		tf::quaternionTFToMsg(q, workspace_pose.pose.orientation);
		workspace_pose.header.frame_id = "/base_footprint";

		workspace_pose.header.stamp = ros::Time::now();

		this->workspace_postion_pub.publish(this->workspace_pose);
	dr_call = boost::bind(&BaseServoController::PIDConfigCallback,this, _1, _2);
			  dr_server.setCallback(dr_call);

}

BaseServoController::~BaseServoController() {
	delete PID_X;
	delete PID_Y;
}


void BaseServoController::PIDConfigCallback(aero_control::BaseServoPIDConfig &config, uint32_t level) {
PID_X->SetPID(config.x_linear_P,config.x_linear_I,config.x_linear_D);
linear_gain = config.x_gain;
PID_Y->SetPID(config.y_linear_P,config.y_linear_I,config.y_linear_D);
rotational_gain = config.y_gain;

this->workspace_pose.pose.position.x = 0;
this->workspace_pose.pose.position.y = 0;
workspace_pose.header.stamp = ros::Time::now();
this->workspace_postion_pub.publish(this->workspace_pose);

}


void BaseServoController::ErrorUpdateTimerCallback(const ros::TimerEvent&) {
	UpdateError();
	if ((ros::Time().now().toSec() - last_position_time.toSec()) > 1) {
		error_update_timer.stop();
		error_update_timer_flag = false;
	}
}

void BaseServoController::DesiredPositionMSG(
		const geometry_msgs::PoseStampedConstPtr& object_pose) {

	tf_listener.waitForTransform("/world", object_pose->header.frame_id, object_pose->header.stamp,
			ros::Duration(0.1));
	tf_listener.transformPose("/world", *object_pose, this->desired_pose);

	last_position_time = ros::Time().now();
	if (error_update_timer_flag == false) {
		this->error_update_timer.start();
		error_update_timer_flag = true;

	}
}




void BaseServoController::UpdateError(void) {
	geometry_msgs::PoseStamped desired_error_pose;
	geometry_msgs::PoseStamped workspace_error_pose;

	tf_listener.waitForTransform("/base_footprint", this->desired_pose.header.frame_id, this->desired_pose.header.stamp,
			ros::Duration(0.1));
	tf_listener.transformPose("/base_footprint", this->desired_pose, desired_error_pose);

	workspace_pose.header.stamp = ros::Time::now();


	tf_listener.waitForTransform("/base_footprint", this->workspace_pose.header.frame_id, this->workspace_pose.header.stamp,
			ros::Duration(0.1));
	tf_listener.transformPose("/base_footprint", this->workspace_pose, workspace_error_pose);


	pos_err.x_err = desired_error_pose.pose.position.x-workspace_error_pose.pose.position.x;
	pos_err.y_err = desired_error_pose.pose.position.y-workspace_error_pose.pose.position.y;

		UpdatePID();
}
void BaseServoController::UpdatePID(void) {

	double forward_vel;

	forward_vel = PID_X->PIDUpdate(pos_err.x_err);
	forward_vel *= linear_gain;

	if (forward_vel > MaxLinearVel()) {
		forward_vel = MaxLinearVel();
	} else if (forward_vel < -MaxLinearVel()) {
		forward_vel = -MaxLinearVel();
	}

	double rotational_vel;

	rotational_vel = PID_Y->PIDUpdate(pos_err.y_err);
	rotational_vel *= rotational_gain;

	if (rotational_vel > MaxAngularVel()) {
		rotational_vel = MaxAngularVel();
	} else if (rotational_vel < -MaxAngularVel()) {
		rotational_vel = -MaxAngularVel();
	}

	geometry_msgs::Twist base_velocity_msg;

	base_velocity_msg.linear.x = forward_vel;

	base_velocity_msg.angular.z = rotational_vel;

}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "base_servo_control");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string DesiredPosition("DesiredPosition"); ///String containing the topic name for goal position
	std::string WorkspacePosition("WorkspacePosition"); ///String containing the topic name for WorkspacePosition
	std::string BaseVelocity("cmd_vel"); ///String containing the topic name for BaseVelocity

	if (argc < 1) {
		ROS_INFO( "Usage: base_servo_control desired_position_topic base_workspace_position_topic base_velocity_topic ");

		return 1;
	} else {
		//Grab the topic parameters, print warnings if using default values
		if (!param_nh.getParam(DesiredPosition, DesiredPosition))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Desired Position Topic <%s>!", DesiredPosition.c_str(), DesiredPosition.c_str());
	if (!param_nh.getParam(BaseVelocity, BaseVelocity))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Base Velocity Topic <%s>!", BaseVelocity.c_str(), BaseVelocity.c_str());

	if (!param_nh.getParam(WorkspacePosition, WorkspacePosition))
				ROS_WARN(
						"Parameter <%s> Not Set. Using Default Workspace Position Topic <%s>!", WorkspacePosition.c_str(), WorkspacePosition.c_str());
		}
//Print out received topics
	ROS_DEBUG("Got Desired Position Topic Name: <%s>", DesiredPosition.c_str());
	ROS_DEBUG("Got Base Velocity Topic Name: <%s>", BaseVelocity.c_str());
	ROS_DEBUG("Got Workspace Position Topic Name: <%s>", WorkspacePosition.c_str());
	ROS_INFO("Starting Up Base Servo Controller...");

//create the arm object
	BaseServoController base_servo(nh, DesiredPosition, WorkspacePosition, BaseVelocity);

	ros::spin();
}

