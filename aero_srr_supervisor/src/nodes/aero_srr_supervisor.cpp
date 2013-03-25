/**
 * @file   aero_srr_supervisor.cpp
 *
 * @date   Mar 22, 2013
 * @author Adam Panzica
 * @brief  Implementation of the aero_srr_supervisor node
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <ros/ros.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_srr_supervisor/Supervisor.h>
//***********    NAMESPACES     ****************//

using namespace aero_srr_supervisor;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aero_srr_supervisor");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	Supervisor supervisor(nh, p_nh);

	ros::spin();
}
