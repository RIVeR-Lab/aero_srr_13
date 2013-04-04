/*
 * arm_controller.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef BASE_SERVO_H_
#define BASE_SERVO_H_




#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <time.h>



namespace visual_servo {

class Base_Servo {
public:
	Base_Servo(ros::NodeHandle nh, std::string ObjectPose,std::string ArmPose);
	void ObjectPosition(const aero_srr_msgs::ObjectLocationMsgConstPtr& object);

private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_object_position;
	ros::Publisher pub_arm_position;
ros::Publisher pub_arm_position_raw;
	ros::Publisher pub_arm_position_trans;

	tf::TransformListener listener;

};

}

#endif /* BASE_SERVO_H_ */
