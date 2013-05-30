/*
 * detectionHandler.cpp
 *
 *  Created on: May 29, 2013
 *      Author: Samir Zutshi
 */


#include <object_locator/detectionHandler.h>

using namespace object_locator;

DetectionHandler::DetectionHandler() :
		gotClassifierMsg_(false),gotColorMsg_(false)

{
	classifier_msg_sub_ = nh_.subscribe("/classiferObjectMsg", 1, &DetectionHandler::classifierCb, this);
	color_msg_sub_		= nh_.subscribe("/colorObjectsMsg",1, &DetectionHandler::colorDetectionCb, this);
	object_pose_pub_ 	= nh_.advertise<aero_srr_msgs::ObjectLocationMsg>("/ObjectPose",2);
	decisionTimer = nh_.createTimer(ros::Duration(1.5),
				&DetectionHandler::decisionMaker, this);
}

DetectionHandler::~DetectionHandler()
{

}

void DetectionHandler::classifierCb(const aero_srr_msgs::ObjectLocationMsgConstPtr& msg)
{
	gotClassifierMsg_ = true;
	classMsg_ = *msg;
	ROS_WARN_STREAM("Got Classifier Msg");

}

void DetectionHandler::colorDetectionCb(const geometry_msgs::PoseArrayConstPtr& msg)
{
	gotColorMsg_ = true;
	colorMsg_ = *msg;
	ROS_WARN_STREAM("Got Color Msg");
}

void DetectionHandler::decisionMaker(const ros::TimerEvent& event)
{
	aero_srr_msgs::ObjectLocationMsg msg;
	geometry_msgs::Pose tempPose;
	if(gotColorMsg_)
	{
		if(gotClassifierMsg_)
		{
			msg.header = classMsg_.header;
			msg.object_type = classMsg_.object_type;
			msg.pose = classMsg_.pose;
			msg.confidence = classMsg_.confidence;
			ROS_ERROR_STREAM("Sent Classifier Msg");

			object_pose_pub_.publish(msg);
		}
		else
		{
			if(!(colorMsg_.poses.empty()))
			{
			tempPose = colorMsg_.poses.front();
			msg.pose.pose = tempPose;
			msg.pose.header = colorMsg_.header;
			ROS_ERROR_STREAM("Sent Color Msg");

			object_pose_pub_.publish(msg);
			}
		}
	}
	else if(gotClassifierMsg_)
	{
		msg.header = classMsg_.header;
		msg.object_type = classMsg_.object_type;
		msg.pose = classMsg_.pose;
		msg.confidence = classMsg_.confidence;
		ROS_ERROR_STREAM("Sent Classifier Msg");
		object_pose_pub_.publish(msg);
	}
	gotClassifierMsg_ = false;
	gotColorMsg_ = false;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "detectionHandler");
	DetectionHandler d;
	ros::spin();
	return 0;
}
