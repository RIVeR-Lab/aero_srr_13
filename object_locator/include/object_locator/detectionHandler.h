/*
 * detectionHandler.h
 *
 *  Created on: May 29, 2013
 *      Author: Samir Zutshi
 */

#ifndef DETECTIONHANDLER_H_
#define DETECTIONHANDLER_H_


#include <object_locator/typedefinitions.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace object_locator
{
	class DetectionHandler
	{

	public:
		DetectionHandler();
		virtual ~DetectionHandler();
		/**
		 * @author Samir Zutshi
		 * @brief Triggered when a detection is published by the lower camera classifier software
		 * 		  and sets a flag saying it has receieved a msg.
		 * @param [in] msg the ObjectLocationMsg published by the lower camera classifier software
		 */
		virtual void classifierCb(const aero_srr_msgs::ObjectLocationMsgConstPtr& msg);
		/**
		 * @author Samir Zutshi
		 * @brief Triggered when a detection is published by the lower camera color detection software
		 * 		  and sets a flag saying it has receieved a msg.
		 * @param [in] msg the ObjectLocationMsg published by the lower camera color detection software
		 */
		virtual void colorDetectionCb(const geometry_msgs::PoseArrayConstPtr& msg);
		/**
		 * @author Samir Zutshi
		 * @brief Triggered when either call back is called to decide which detection to push through,
		 * 		  then creates a msg and publishes it.
		 */
		virtual void decisionMaker(const ros::TimerEvent& event);
	private:
		ros::Subscriber classifier_msg_sub_, color_msg_sub_;
		ros::NodeHandle nh_;
		ros::Publisher object_pose_pub_;
		ros::Timer decisionTimer;


		bool gotClassifierMsg_, gotColorMsg_;
		aero_srr_msgs::ObjectLocationMsg classMsg_;
		geometry_msgs::PoseArray colorMsg_;

	};
}


#endif /* DETECTIONHANDLER_H_ */
