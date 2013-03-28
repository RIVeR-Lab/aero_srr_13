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



namespace aero_arm {

class Arm_Controller {
public:
	Arm_Controller(ros::NodeHandle nh, std::string ObjectPose,std::string ArmPose);
	void ObjectPosition(const aero_srr_msgs::ObjectLocationMsgConstPtr& object);

private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_object_position;
	ros::Publisher pub_arm_position;
	tf::TransformListener listener;


};

}

#endif /* ARM_CONTROLLER_H_ */
