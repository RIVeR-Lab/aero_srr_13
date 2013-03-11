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
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//
namespace aero_base
{

class AeroBase
{
public:
	AeroBase(ros::NodeHandle& nh, ros::NodeHandle& p_nh);

private:
	ros::NodeHandle nh_;
	ros::NodeHandle p_nh_;
};

};

#endif /* AEROBASE_H_ */
