/*
 * test_tf_tree.cpp
 *
 *  Created on: Mar 25, 2013
 *      Author: srr
 */


#include<ros/ros.h>
#include<tf/transform_broadcaster.h>

tf::TransformBroadcaster* broadcast;
void broadcastTimer(const ros::TimerEvent& event)
{
	tf::Transform tf;
	tf.setOrigin(tf::Vector3(1, 1, 1));
	tf.setRotation(tf::Quaternion(0, 0, 0));
	broadcast->sendTransform(tf::StampedTransform(tf, ros::Time(0),  "/stereo_bottom/center", "/world"));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_tf_tree");
	ros::NodeHandle nh;
	broadcast = new tf::TransformBroadcaster();
	ros::Timer timer = nh.createTimer(ros::Duration(1/100), broadcastTimer);
	ros::spin();
}
