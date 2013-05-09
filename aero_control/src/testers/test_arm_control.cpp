//============================================================================
// Name        : Jaco.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

/**
 * @file jaco_arm_control.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File
#include <aero_control/arm_controller.h>
#include <dynamic_reconfigure/server.h>
#include <aero_control/TestVelocityConfig.h>

using namespace std;
ros::Publisher pub;
tf::TransformListener *listenerptr;

float x_pos = 0.4;
float y_pos = 0;
float z_pos = 0;
float rx_pos = 0;
float ry_pos = 0;
float rz_pos = 0;

void TimerCallback(const ros::TimerEvent&) {
	aero_srr_msgs::ObjectLocationMsg test_msg;

	test_msg.pose.pose.position.x = x_pos;
	test_msg.pose.pose.position.y = y_pos;
	test_msg.pose.pose.position.z = z_pos;

	tf::Quaternion q;

	q.setRPY(rx_pos, ry_pos, rz_pos);

	tf::quaternionTFToMsg(q, test_msg.pose.pose.orientation);
	test_msg.header.frame_id = "/arm_base";
	test_msg.pose.header.frame_id = test_msg.header.frame_id;
	test_msg.header.stamp = ros::Time::now();
	test_msg.pose.header.stamp = ros::Time::now();
	pub.publish(test_msg);

}
void TimerCallback2(const ros::TimerEvent&) {

	geometry_msgs::PoseStamped end_effector_pose;
	geometry_msgs::PoseStamped arm_pose;

	tf::Quaternion q;
	end_effector_pose.pose.position.x = 0;
	end_effector_pose.pose.position.y = 0;
	end_effector_pose.pose.position.z = 0;
	q.setRPY(0, 0, 0);

	tf::quaternionTFToMsg(q, end_effector_pose.pose.orientation);
	end_effector_pose.header.frame_id = "/jaco_end_effector";

	end_effector_pose.header.stamp = ros::Time::now();

	listenerptr->waitForTransform("/world", end_effector_pose.header.frame_id, end_effector_pose.header.stamp, ros::Duration(0.5));

	listenerptr->transformPose("/world", end_effector_pose, arm_pose);

	ROS_INFO("X = %f", arm_pose.pose.position.x);
	ROS_INFO("Y = %f", arm_pose.pose.position.y);
	ROS_INFO("Z = %f", arm_pose.pose.position.z);

	listenerptr->waitForTransform("/jaco_api_origin", end_effector_pose.header.frame_id, end_effector_pose.header.stamp, ros::Duration(0.5));

	listenerptr->transformPose("/jaco_api_origin", end_effector_pose, arm_pose);

	ROS_INFO("API X = %f", arm_pose.pose.position.x);
	ROS_INFO("API Y = %f", arm_pose.pose.position.y);
	ROS_INFO("API Z = %f", arm_pose.pose.position.z);

}

void callback(aero_control::TestVelocityConfig &config, uint32_t level) {

	x_pos = config.X_Position;
	y_pos = config.Y_Position;
	z_pos = config.Z_Position;
	rx_pos = config.rX_Position;
	ry_pos = config.rY_Position;
	rz_pos = config.rZ_Position;

}
int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "test_arm_control");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string ObjectPose("ObjectPose"); ///String containing the topic name for cartesian commands

	pub = nh.advertise<aero_srr_msgs::ObjectLocationMsg>(ObjectPose, 2);
	tf::TransformListener listener;
	listenerptr = &listener;

	ros::Timer timer = nh.createTimer(ros::Duration(0.1), TimerCallback);

	//ros::Timer timer2 = nh.createTimer(ros::Duration(1.0),TimerCallback2);

	dynamic_reconfigure::Server<aero_control::TestVelocityConfig> server;
	dynamic_reconfigure::Server<aero_control::TestVelocityConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	ros::spin();
}

