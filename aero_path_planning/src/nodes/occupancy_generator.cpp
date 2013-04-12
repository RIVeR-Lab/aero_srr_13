/**
 * @file	occupancy_generator.cpp
 * @date	Oct 25, 2012
 * @author	Adam Panzica
 * @brief	simple node for generating test occupancy grids
 */
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<aero_srr_msgs/SoftwareStop.h>
#include<aero_path_planning/OccupancyGridMsg.h>
#include<aero_path_planning/utilities/OccupancyGrid.h>

///Macro for printing out warning messages if default parameters are used
#define PARAM_WARN(param,value) ROS_WARN(warn_message.c_str(), param.c_str(), value.c_str())

int main(int argc, char **argv)
{
	std::string warn_message("Parameter <%s> Not Set. Using Default Value <%s>");
	ros::init(argc, argv, "Occupancy_Generator");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<aero_path_planning::OccupancyGridMsg>("aero/occupancy_point_cloud_topic", 2);
	ros::Publisher s_pub = nh.advertise<aero_srr_msgs::SoftwareStop>("aero/software_stop", 2);

	//x dimension of occupancy grid
	std::string p_x_dim("occupancy/x_dimension");
	double x_dim = 20;
	std::string x_dim_msg("");
	x_dim_msg+= boost::lexical_cast<double>(x_dim);
	x_dim_msg+="m";

	//y dimension of occupancy grid
	std::string p_y_dim("occupancy/y_dimension");
	double y_dim = 20;
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
	double res = .25;
	std::string p_res_msg("");
	p_res_msg+= boost::lexical_cast<double>(res);
	p_res_msg+="m";

	if(!nh.getParam(p_x_dim,	x_dim))			PARAM_WARN(p_x_dim,		x_dim_msg);
	if(!nh.getParam(p_y_dim,	y_dim))			PARAM_WARN(p_y_dim,		y_dim_msg);
	if(!nh.getParam(p_z_dim,	z_dim))			PARAM_WARN(p_z_dim,		z_dim_msg);
	if(!nh.getParam(p_res,		res))			PARAM_WARN(p_res,		p_res_msg);

	aero_path_planning::Point origin;
	origin.x = 0;
	origin.y = y_dim/2;
	origin.z = 0;

	bool stop = true;
	while(ros::ok())
	{
		aero_path_planning::OccupancyGrid grid(x_dim, y_dim, res, origin, aero_path_planning::FREE_LOW_COST);
		aero_path_planning::PointCloud line_cloud;
		aero_path_planning::Point start_point;
		start_point.z=0;
		start_point.rgba = aero_path_planning::OBSTACLE;
		aero_path_planning::Point end_point;
		end_point.z=0;
		end_point.rgba = aero_path_planning::OBSTACLE;

		aero_path_planning::Point goal_point;

		ROS_INFO("Enter X0: ");
		std::cin >> start_point.x;
		start_point.x=std::floor(start_point.x);
		ROS_INFO("I Got Input!: %f", start_point.x);
		ROS_INFO("Enter Y0: ");
		std::cin >> start_point.y;
		start_point.y=std::floor(start_point.y);
		ROS_INFO("I Got Input!: %f", start_point.y);

		ROS_INFO("Enter Xf: ");
		std::cin >> end_point.x;
		end_point.x=std::floor(end_point.x);
		ROS_INFO("I Got Input!: %f", end_point.x);
		ROS_INFO("Enter Yf: ");
		std::cin >> end_point.y;
		end_point.y=std::floor(end_point.y);
		ROS_INFO("I Got Input!: %f", end_point.y);

		ROS_INFO("Enter Goal X: ");
		std::cin >> goal_point.x;
		goal_point.x=std::floor(goal_point.x);
		ROS_INFO("I Got Input!: %f", end_point.x);
		ROS_INFO("Enter Goal Y: ");
		std::cin >> goal_point.y;
		goal_point.y=std::floor(goal_point.y);
		ROS_INFO("I Got Input!: %f", end_point.y);

		goal_point.z = 0;

		start_point.getVector4fMap();
		end_point.getVector4fMap();

		aero_path_planning::castLine(start_point, end_point, aero_path_planning::OBSTACLE, line_cloud);

		for(aero_path_planning::PointCloud::iterator line_itr = line_cloud.begin(); line_itr<line_cloud.end(); line_itr++)
		{
			PRINT_POINT("Line Point", (*line_itr));
			grid.setPointTrait(*line_itr, (aero_path_planning::PointTrait)line_itr->rgba);
		}

		ROS_INFO("I'm Sending Occupancy Grid:\n%s", grid.toString(0,0)->c_str());
		try
		{
			grid.setGoalPoint(goal_point);
		}
		catch(std::runtime_error& e)
		{
			ROS_WARN_STREAM(e.what());
		}
		aero_path_planning::OccupancyGridMsg message;
		message.header.frame_id = "base_link";
		grid.generateMessage(message);
		pub.publish(message);
	}
}


