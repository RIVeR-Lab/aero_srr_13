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
#include <jaco_driver/finger_position.h>



namespace aero_control {

class ArmController {
public:
	ArmController(ros::NodeHandle nh, ros::NodeHandle param_nh);
	void ObjectPositionMSG(const aero_srr_msgs::ObjectLocationMsgConstPtr& object);
	void AeroStateMSG(const aero_srr_msgs::AeroState& aero_state);
private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_object_position;
	ros::Publisher pub_arm_position;
	ros::Publisher pub_set_finger_position;
	ros::Subscriber aero_state_sub;
	ros::ServiceClient aero_state_transition_srv_client;

	tf::TransformListener listener;

	bool active_state;
	uint8_t previous_state;

};

}

#endif /* ARM_CONTROLLER_H_ */
