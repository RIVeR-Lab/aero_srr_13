/**
 * @file	oryx_global_planner.cpp
 * @date	Jan 1, 2013
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

//*********************SYSTEM DEPENDENCIES**********************//
#include<ros/ros.h>

//*********************LOCAL DEPENDENCIES**********************//
#include"OryxPathPlanning.h"
#include"OryxPathPlannerConfig.h"


using namespace oryx_path_planning;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "oryx_global_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");


	//Comunication Parameters
	std::string local_planner_topic("occupancy_point_cloud_topic");
	std::string odometry_topic("odometry_topic");

	//Configuration Parameters
	//*****************Configuration Parameters*******************//
	//Minimum update rate expected of occupancy grid
	std::string p_up_rate("occupancy/update_rate");
	double update_rate = 0.2;
	std::string up_rate_msg("");
	up_rate_msg+= boost::lexical_cast<double>(update_rate);
	up_rate_msg+="s";

	//x dimension of occupancy grid
	std::string p_x_dim("occupancy/x_dimension");
	double x_dim = 200;
	std::string x_dim_msg("");
	x_dim_msg+= boost::lexical_cast<double>(x_dim);
	x_dim_msg+="m";

	//y dimension of occupancy grid
	std::string p_y_dim("occupancy/y_dimension");
	double y_dim = 200;
	std::string y_dim_msg("");
	y_dim_msg+= boost::lexical_cast<double>(y_dim);
	y_dim_msg+="m";

	//z dimension of occupancy grid
	std::string p_z_dim("occupancy/z_dimension");
	double z_dim = 0;
	std::string z_dim_msg("");
	z_dim_msg+= boost::lexical_cast<double>(z_dim);
	z_dim_msg+="m";

	//resolution occupancy grid
	std::string p_res("occupancy/grid_resolution");
	double res = .01;
	std::string p_res_msg("");
	p_res_msg+= boost::lexical_cast<double>(res);
	p_res_msg+="m";

	//x coord of the origin of the occupancy grids
	std::string p_x_ori("occupancy/x_origin");
	double x_ori = 0;
	std::string p_x_ori_msg("");
	p_x_ori_msg+= boost::lexical_cast<double>(x_ori);
	p_x_ori_msg+="m";

	//z coord of the origin of the occupancy grids
	std::string p_z_ori("occupancy/z_origin");
	double z_ori = 0;
	std::string p_z_ori_msg("");
	p_z_ori_msg+= boost::lexical_cast<double>(z_ori);
	p_z_ori_msg+="m";

	//y coord of the origin of the occupancy grids
	std::string p_y_ori("occupancy/y_origin");
	double y_ori = y_dim/2;
	std::string p_y_ori_msg("");
	p_y_ori_msg+= boost::lexical_cast<double>(y_ori);
	p_y_ori_msg+="m";

	//Node Information Printout
	ROS_INFO("Starting Up Oryx Global Planner Version %d.%d.%d", oryx_path_planner_VERSION_MAJOR, oryx_path_planner_VERSION_MINOR, oryx_path_planner_VERSION_BUILD);

	//Get Private Parameters
	if(!p_nh.getParam(local_planner_topic,local_planner_topic))	PARAM_WARN(local_planner_topic,	local_planner_topic);
	if(!p_nh.getParam(odometry_topic,	odometry_topic))		PARAM_WARN(odometry_topic,		odometry_topic);

	//Get Public Parameters
	if(!nh.getParam(p_up_rate,	update_rate))	PARAM_WARN(p_up_rate,	up_rate_msg);
	if(!nh.getParam(p_x_dim,	x_dim))			PARAM_WARN(p_x_dim,		x_dim_msg);
	if(!nh.getParam(p_y_dim,	y_dim))			PARAM_WARN(p_y_dim,		y_dim_msg);
	if(!nh.getParam(p_z_dim,	z_dim))			PARAM_WARN(p_z_dim,		z_dim_msg);
	if(!nh.getParam(p_x_ori,	x_ori))			PARAM_WARN(p_x_ori,		p_x_ori_msg);
	if(!nh.getParam(p_y_ori,	y_ori))			PARAM_WARN(p_y_ori,		p_y_ori_msg);
	if(!nh.getParam(p_z_ori,	z_ori))			PARAM_WARN(p_z_ori,		p_z_ori_msg);
	if(!nh.getParam(p_res,		res))			PARAM_WARN(p_res,		p_res_msg);

	ROS_INFO("Global Planner Configuration Parameters Set...");


}
