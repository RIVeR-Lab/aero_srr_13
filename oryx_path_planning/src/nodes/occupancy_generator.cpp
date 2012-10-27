/**
 * @file	occupancy_generator.cpp
 * @date	Oct 25, 2012
 * @author	Adam Panzica
 * @brief	simple node for generating test occupancy grids
 */
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<oryxsrr_msgs/SoftwareStop.h>
#include"OccupancyGrid.h"

///Macro for printing out warning messages if default parameters are used
#define PARAM_WARN(param,value) ROS_WARN(warn_message.c_str(), param.c_str(), value.c_str())

int main(int argc, char **argv) {
	std::string warn_message("Parameter <%s> Not Set. Using Default Value <%s>");
	ros::init(argc, argv, "Occupancy_Generator");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_point_cloud_topic", 2);
	ros::Publisher s_pub = nh.advertise<oryxsrr_msgs::SoftwareStop>("oryx/software_stop", 2);

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
		double input;
		ROS_INFO("Enter A Number: ");
		std::cin >> input;
		ROS_INFO("I Got Input!: %f", input);
		oryx_path_planning::OccupancyGrid grid(xDim, yDim, res, origin, oryx_path_planning::FREE_LOW_COST);
		if(input>0){
			double y;
			double x;
			int numPoints = 0;
			for(x=0; x<xDim; x+=res/20){
				y= input*x;
				if(y>(yDim/2)||y<-yDim/2){
					break;
				}
				try{
					ROS_INFO("Setting Point at <%f,%f>", x,y);
					grid.setPointTrait(x,y-origin.y,0,oryx_path_planning::OBSTACLE);
					numPoints++;
				}catch(std::exception& e){
					ROS_ERROR("%s", e.what());
				}
			}
			ROS_INFO("I'm Sending Occupancy Grid:\n%s", grid.toString(0,0)->c_str());
			ROS_INFO("I placed ~%d points", numPoints);
			sensor_msgs::PointCloud2Ptr message(new sensor_msgs::PointCloud2());
			message->header.frame_id = "base_link";
			grid.generateMessage(message);
			pub.publish(message);
		}else{
			oryxsrr_msgs::SoftwareStop stopMessage;
			stopMessage.stop = stop;
			stopMessage.message = "I'm testing Software Stop";
			s_pub.publish(stopMessage);
			if(stop) stop = false;
			else stop = true;
		}
	}
}


