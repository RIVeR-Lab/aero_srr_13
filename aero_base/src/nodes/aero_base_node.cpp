/**
 * @file   aero_base_node.cpp
 *
 * @date   Mar 11, 2013
 * @author Adam Panzica
 * @brief  Implementation for the aero_base_node ROS node
 */

//*********** SYSTEM DEPENDANCIES ****************//
//************ LOCAL DEPENDANCIES ****************//
#include <aero_base/AeroBase.h>
//***********    NAMESPACES     ****************//

using namespace aero_base;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aero_base_node");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh;

	AeroBase aero_base(nh, p_nh);

	ros::spin();
}
