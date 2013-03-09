/**
 * @file	oryx_base_planner.cpp
 * @date	Oct 23, 2012
 * @author	Adam Panzica
 * @brief	Planning node for doing local planning and movement operations
 */

//*********************** SYSTEM DEPENDENCIES ************************************//
#include<boost/lexical_cast.hpp>
//*********************** LOCAL DEPENDENCIES ************************************//
#include<aero_path_planning/LocalPlanner.h>

using namespace aero_path_planning;

//*********************** HELPER CLASS DEFINITIONS ******************************//



//*********************** NODE IMPLEMENTATION ******************************//

int main(int argc, char **argv) {
	ros::init(argc, argv, "aero_base_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	//Set up client to Drive Controller
	try
	{
		LocalPlanner planner(nh, p_nh);
		planner.doPlanning();
	}
	catch(std::exception& e)
	{
		ROS_FATAL("%s, %s",e.what(), "Shutting Down");
		ros::shutdown();
	}
}


//};
