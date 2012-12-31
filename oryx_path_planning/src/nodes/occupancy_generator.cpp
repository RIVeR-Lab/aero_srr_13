/**
 * @file	occupancy_generator.cpp
 * @date	Oct 25, 2012
 * @author	Adam Panzica
 * @brief	simple node for generating test occupancy grids
 */
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<oryx_msgs/SoftwareStop.h>
#include"oryx_path_planning/OccupancyGridMsg.h"
#include"OccupancyGrid.h"

///Macro for printing out warning messages if default parameters are used
#define PARAM_WARN(param,value) ROS_WARN(warn_message.c_str(), param.c_str(), value.c_str())

int main(int argc, char **argv) {
	std::string warn_message("Parameter <%s> Not Set. Using Default Value <%s>");
	ros::init(argc, argv, "Occupancy_Generator");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<oryx_path_planning::OccupancyGridMsg>("occupancy_point_cloud_topic", 2);
	ros::Publisher s_pub = nh.advertise<oryx_msgs::SoftwareStop>("oryx/software_stop", 2);

	//x dimension of occupancy grid
	std::string p_x_dim("occupancy/x_dimension");
	double xDim = 20;
	std::string xDim_msg("");
	xDim_msg+= boost::lexical_cast<double>(xDim);
	xDim_msg+="m";

	//y dimension of occupancy grid
	std::string p_y_dim("occupancy/y_dimension");
	double yDim = 20;
	std::string yDim_msg("");
	yDim_msg+= boost::lexical_cast<double>(yDim);
	yDim_msg+="m";

	//z dimension of occupancy grid
	std::string p_z_dim("occupancy/z_dimension");
	double zDim = 0;
	std::string zDim_msg("");
	zDim_msg+= boost::lexical_cast<double>(zDim);
	zDim_msg+="m";

	//resolution occupancy grid
	std::string p_res("occupancy/grid_resolution");
	double res = .25;
	std::string p_res_msg("");
	p_res_msg+= boost::lexical_cast<double>(res);
	p_res_msg+="m";

	if(!nh.getParam(p_x_dim,	xDim))			PARAM_WARN(p_x_dim,		xDim_msg);
	if(!nh.getParam(p_y_dim,	yDim))			PARAM_WARN(p_y_dim,		yDim_msg);
	if(!nh.getParam(p_z_dim,	zDim))			PARAM_WARN(p_z_dim,		zDim_msg);
	if(!nh.getParam(p_res,		res))			PARAM_WARN(p_res,		p_res_msg);

	oryx_path_planning::Point origin;
	origin.x = 0;
	origin.y = yDim/2;
	origin.z = 0;

	bool stop = true;
	while(ros::ok()){
		oryx_path_planning::OccupancyGrid grid(xDim, yDim, res, origin, oryx_path_planning::FREE_LOW_COST);
		oryx_path_planning::PointCloud lineCloud;
		oryx_path_planning::Point startPoint;
		startPoint.z=0;
		startPoint.rgba = oryx_path_planning::OBSTACLE;
		oryx_path_planning::Point endPoint;
		endPoint.z=0;
		endPoint.rgba = oryx_path_planning::OBSTACLE;

		ROS_INFO("Enter X0: ");
		std::cin >> startPoint.x;
		startPoint.x=std::floor(startPoint.x);
		ROS_INFO("I Got Input!: %f", startPoint.x);
		ROS_INFO("Enter Y0: ");
		std::cin >> startPoint.y;
		startPoint.y=std::floor(startPoint.y);
		ROS_INFO("I Got Input!: %f", startPoint.y);

		ROS_INFO("Enter Xf: ");
		std::cin >> endPoint.x;
		endPoint.x=std::floor(endPoint.x);
		ROS_INFO("I Got Input!: %f", endPoint.x);
		ROS_INFO("Enter Yf: ");
		std::cin >> endPoint.y;
		endPoint.y=std::floor(endPoint.y);
		ROS_INFO("I Got Input!: %f", endPoint.y);

		startPoint.getVector4fMap();
		endPoint.getVector4fMap();

		oryx_path_planning::castLine(startPoint, endPoint, oryx_path_planning::OBSTACLE, lineCloud);

		for(oryx_path_planning::PointCloud::iterator line_itr = lineCloud.begin(); line_itr<lineCloud.end(); line_itr++){
			PRINT_POINT("Line Point", (*line_itr));
			grid.setPointTrait(*line_itr, (oryx_path_planning::PointTrait)line_itr->rgba);
		}

		ROS_INFO("I'm Sending Occupancy Grid:\n%s", grid.toString(0,0)->c_str());
		oryx_path_planning::OccupancyGridMsg message;
		message.header.frame_id = "base_link";
		grid.generateMessage(message);
		pub.publish(message);
	}
}


