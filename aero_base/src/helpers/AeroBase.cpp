/**
 * @file   AeroBase.cpp
 *
 * @date   Mar 11, 2013
 * @author Adam Panzica
 * @brief  Implementation of AeroBase class
 */

//*********** SYSTEM DEPENDANCIES ****************//
//************ LOCAL DEPENDANCIES ****************//
#include <aero_base/AeroBase.h>
//***********    NAMESPACES     ****************//


using namespace aero_base;

AeroBase::AeroBase(ros::NodeHandle& nh, ros::NodeHandle& p_nh):
		nh_(nh),
		p_nh_(p_nh)
{

}
