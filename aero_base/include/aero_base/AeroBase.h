/**
 * @file   AeroBase.h
 *
 * @date   Mar 6, 2013
 * @author parallels
 * @brief  \TODO
 */

#ifndef AEROBASE_H_
#define AEROBASE_H_

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//
namespace aero_base
{

class AeroBase
{
public:
	AeroBase(ros::NodeHandle& nh, ros::NodeHandle& p_nh);

private:

	void loadParams();
	void registerTimers();
	void buildTransforms();

	/**
	 * @author Adam Panzica
	 * @brief Timer callback that updates the transforms
	 * @param event Timer event with skew info
	 */
	void updateCB(const ros::TimerEvent& event);

	ros::NodeHandle nh_;
	ros::NodeHandle p_nh_;

	//Timer to update the transforms
	ros::Timer      update_timer_;
	std::string     robot_;

	//Lower Camera Transforms
	std::string left_cam_;
	tf::TransformBroadcaster to_left_lower_camera_bc_;
	tf::Transform            to_left_lower_camera_;
	std::string right_cam_;
	tf::TransformBroadcaster to_right_lower_camera_bc_;
	tf::Transform            to_right_lower_camera_;

	//Arm Base Transform
	std::string arm_base_;
	tf::TransformBroadcaster to_arm_base_bc_;
	tf::Transform            to_arm_base_;

	//Boom Base Transform
	std::string boom_;
	tf::TransformBroadcaster to_boom_base_bc_;
	tf::Transform            to_boom_base_;

	//IMU Base transform
	std::string imu_;
	tf::TransformBroadcaster to_imu_bc_;
	tf::Transform            to_imu_;

	//LIDAR Base transform
	std::string lidar_;
	tf::TransformBroadcaster to_lidar_bc_;
	tf::Transform            to_lidar_;
};

};

#endif /* AEROBASE_H_ */
