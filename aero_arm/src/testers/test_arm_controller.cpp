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
#include <aero_arm/arm_controller.h>

using namespace std;
ros::Publisher pub;
tf::TransformListener *listenerptr;

void TimerCallback(const ros::TimerEvent&)
{
	aero_srr_msgs::ObjectLocationMsg test_msg;

			test_msg.pose.pose.position.x = 0.45;
			test_msg.pose.pose.position.y = 0.2;
			test_msg.pose.pose.position.z = 0.1;

			tf::Quaternion q;

			q.setRPY(0,0,0);

			tf::quaternionTFToMsg(q,test_msg.pose.pose.orientation);
			test_msg.header.frame_id = "/arm_base";
			test_msg.pose.header.frame_id = test_msg.header.frame_id;
			test_msg.header.stamp = ros::Time::now();
				test_msg.pose.header.stamp = ros::Time::now();
			pub.publish(test_msg);





}
void TimerCallback2(const ros::TimerEvent&)
{

	geometry_msgs::PoseStamped end_effector_pose;
	geometry_msgs::PoseStamped arm_pose;

	tf::Quaternion q;
	end_effector_pose.pose.position.x = 0;
	end_effector_pose.pose.position.y = 0;
	end_effector_pose.pose.position.z = 0;
				q.setRPY(0,0,0);

				tf::quaternionTFToMsg(q,end_effector_pose.pose.orientation);
	end_effector_pose.header.frame_id = "/jaco_end_effector";

	end_effector_pose.header.stamp = ros::Time::now();


	listenerptr->waitForTransform("/arm_base", end_effector_pose.header.frame_id, end_effector_pose.header.stamp, ros::Duration(0.5) );

	listenerptr->transformPose("/arm_base", end_effector_pose, arm_pose);

		ROS_INFO("X = %f",arm_pose.pose.position.x);
		ROS_INFO("Y = %f", arm_pose.pose.position.y);
		ROS_INFO("Z = %f", arm_pose.pose.position.z);

		listenerptr->waitForTransform("/jaco_api_origin", end_effector_pose.header.frame_id, end_effector_pose.header.stamp, ros::Duration(0.5) );

		listenerptr->transformPose("/jaco_api_origin", end_effector_pose, arm_pose);

			ROS_INFO("API X = %f",arm_pose.pose.position.x);
			ROS_INFO("API Y = %f", arm_pose.pose.position.y);
			ROS_INFO("API Z = %f", arm_pose.pose.position.z);


}
int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "test_arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string Object("Object"); ///String containing the topic name for cartesian commands

	 pub = nh.advertise<aero_srr_msgs::ObjectLocationMsg>(Object,
			2);
	 tf::TransformListener listener;
	 listenerptr = &listener;

	ros::Timer timer = nh.createTimer(ros::Duration(10.0),TimerCallback);

	ros::Timer timer2 = nh.createTimer(ros::Duration(1.0),TimerCallback2);

	ros::spin();
}

