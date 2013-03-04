/**
 * @file global_planner.cpp
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//

//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/GlobalPlanner.h>
#include<aero_path_planning/RRTCarrot.h>
#include<aero_path_planning/OccupancyGridMsg.h>
//**********************NAMESPACES*****************************//


using namespace aero_path_planning;

GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh, aero_path_planning::CarrotPathFinder& path_planner):
		state_(MANUAL),
		path_planner_(&path_planner),
		nh_(nh),
		p_nh_(p_nh)
{
	ROS_INFO("Initializing Global Planner...");


	ROS_INFO("Loading Occupancy Grid Parameters...");
	this->loadOccupancyParam();
	ROS_INFO("Registering Topics...");
	this->registerTopics();

	ROS_INFO("Aero Global Planner Running!");

}

GlobalPlanner::~GlobalPlanner()
{

}

void GlobalPlanner::loadOccupancyParam()
{
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

	//Get Public Parameters
	if(!this->nh_.getParam(p_up_rate,	update_rate))	PARAM_WARN(p_up_rate,	up_rate_msg);
	if(!this->nh_.getParam(p_x_dim,	x_dim))				PARAM_WARN(p_x_dim,		x_dim_msg);
	if(!this->nh_.getParam(p_y_dim,	y_dim))				PARAM_WARN(p_y_dim,		y_dim_msg);
	if(!this->nh_.getParam(p_z_dim,	z_dim))				PARAM_WARN(p_z_dim,		z_dim_msg);
	if(!this->nh_.getParam(p_x_ori,	x_ori))				PARAM_WARN(p_x_ori,		p_x_ori_msg);
	if(!this->nh_.getParam(p_y_ori,	y_ori))				PARAM_WARN(p_y_ori,		p_y_ori_msg);
	if(!this->nh_.getParam(p_z_ori,	z_ori))				PARAM_WARN(p_z_ori,		p_z_ori_msg);
	if(!this->nh_.getParam(p_res,	res))				PARAM_WARN(p_res,		p_res_msg);

	this->local_x_size_ = x_dim;
	this->local_y_size_ = y_dim;
}

void GlobalPlanner::registerTopics()
{
	//Comunication Parameters
	std::string local_planner_topic("aero/occupancy_point_cloud_topic");
	std::string odometry_topic("odometry_topic");
	std::string command_topic("/global_planning/commands");


	this->laser_topic_ = "/laser";


	//Get Private Parameters
	if(!this->p_nh_.getParam(local_planner_topic,this->local_occupancy_topic_))	PARAM_WARN(local_planner_topic,	local_planner_topic);
	if(!this->p_nh_.getParam(odometry_topic,	odometry_topic))		PARAM_WARN(odometry_topic,		odometry_topic);

	this->local_occ_pub_ = this->nh_.advertise<aero_path_planning::OccupancyGridMsg>(this->local_occupancy_topic_, 2);
	this->laser_sub_     = this->nh_.subscribe(this->laser_topic_, 2, &GlobalPlanner::laserCB, this);
}

void GlobalPlanner::laserCB(const sensor_msgs::PointCloud2ConstPtr message)
{
	PointCloud scan_cloud;
	Point origin;
	origin.x = 0;
	origin.y = this->local_y_size_/2;
	origin.z = 0;
	OccupancyGrid local_map(this->local_x_size_, this->local_y_size_, .01, origin, aero_path_planning::UNKNOWN);
	pcl::fromROSMsg(*message, scan_cloud);
	aero_path_planning::PointConverter converter(.01);
	BOOST_FOREACH(Point point, scan_cloud)
	{
		converter.convertToGrid(point, point);
		point.z = 0;
		Point on_grid;
		on_grid.getVector4fMap() = point.getVector4fMap()+origin.getVector4fMap();
		if(on_grid.x>=0&&on_grid.x<this->local_x_size_&&on_grid.y>0&&on_grid.y<this->local_y_size_)
		{
			local_map.setPointTrait(point, aero_path_planning::OBSTACLE);
		}
	}

	aero_path_planning::OccupancyGridMsg message_out;
	local_map.generateMessage(message_out);
	this->local_occ_pub_.publish(message_out);

}
