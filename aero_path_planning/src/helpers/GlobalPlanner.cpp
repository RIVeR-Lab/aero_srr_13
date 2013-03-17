/**
 * @file GlobalPlanner.cpp
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<boost/foreach.hpp>
#include<pcl_ros/transforms.h>
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
				transformer_(nh),
				p_nh_(p_nh)
{
	ROS_INFO("Initializing Global Planner...");


	ROS_INFO("Loading Occupancy Grid Parameters...");
	this->loadOccupancyParam();
	ROS_INFO("Registering Topics...");
	this->registerTopics();

	ROS_INFO("Global Planner Running!");

}

GlobalPlanner::~GlobalPlanner()
{

}

void GlobalPlanner::loadOccupancyParam()
{
	//Configuration Parameters
	//*****************Configuration Parameters*******************//
	//Minimum update rate expected of occupancy grid
	std::string p_up_rate(L_OCC_UPDTRT);
	this->local_update_rate_ = 1.0/40.0;
	std::stringstream up_rate_msg;
	up_rate_msg<<this->local_update_rate_<<"s";

	//x dimension of occupancy grid
	std::string p_x_dim(L_OCC_XDIM);
	this->local_x_size_ = 200;
	std::stringstream x_dim_msg;
	x_dim_msg<<this->local_x_size_<<"m";

	//y dimension of occupancy grid
	std::string p_y_dim(L_OCC_YDIM);
	this->local_y_size_ = 200;
	std::stringstream y_dim_msg;
	y_dim_msg<<this->local_y_size_<<"m";

	//z dimension of occupancy grid
	std::string p_z_dim(L_OCC_ZDIM);
	this->local_z_size_ = 0;
	std::stringstream z_dim_msg;
	z_dim_msg<<this->local_z_size_<<"m";

	//resolution occupancy grid
	std::string p_res(L_OCC_RES);
	this->local_res_ = .01;
	std::stringstream p_res_msg;
	p_res_msg<<this->local_res_<<"m";

	//x coord of the origin of the occupancy grids
	std::string p_x_ori(L_OCC_XORG);
	this->local_x_ori_ = 0;
	std::stringstream p_x_ori_msg;
	p_x_ori_msg<<this->local_x_ori_<<"m";

	//y coord of the origin of the occupancy grids
	std::string p_y_ori(L_OCC_YORG);
	this->local_y_ori_ = this->local_y_size_/2;
	std::stringstream p_y_ori_msg;
	p_y_ori_msg<<this->local_y_ori_<<"m";

	//z coord of the origin of the occupancy grids
	std::string p_z_ori(L_OCC_ZORG);
	this->local_z_ori_ = 0;
	std::stringstream p_z_ori_msg;
	p_z_ori_msg<<this->local_z_ori_<<"m";

	//x dimension of global occupancy grid
	std::string pg_x_dim(G_OCC_XDIM);
	this->global_x_size_ = 5000;
	std::stringstream gx_dim_msg;
	gx_dim_msg<<this->global_x_size_<<"m";

	//y dimension of global occupancy grid
	std::string pg_y_dim(G_OCC_YDIM);
	this->global_y_size_ = 5000;
	std::stringstream gy_dim_msg;
	gy_dim_msg<<this->global_y_size_<<"m";

	//z dimension of global occupancy grid
	std::string pg_z_dim(G_OCC_ZDIM);
	this->global_z_size_ = 0;
	std::stringstream gz_dim_msg;
	gz_dim_msg<<this->global_z_size_<<"m";

	//resolution global occupancy grid
	std::string pg_res(G_OCC_RES);
	this->global_res_ = .01;
	std::stringstream pg_res_msg;
	pg_res_msg<<this->global_res_<<"m";

	//x coord of the global origin of the occupancy grids
	std::string pg_x_ori(G_OCC_XORG);
	this->global_x_ori_ = 0;
	std::stringstream pg_x_ori_msg;
	pg_x_ori_msg<<this->global_x_ori_<<"m";

	//y coord of the global origin of the occupancy grids
	std::string pg_y_ori(G_OCC_YORG);
	this->global_y_ori_ = 0;
	std::stringstream pg_y_ori_msg;
	pg_y_ori_msg<<this->global_y_ori_<<"m";

	//z coord of the global origin of the occupancy grids
	std::string pg_z_ori(G_OCC_ZORG);
	this->global_z_ori_ = 0;
	std::stringstream pg_z_ori_msg;
	pg_z_ori_msg<<this->global_z_ori_<<"m";

	//Get Public Parameters
	if(!this->nh_.getParam(p_up_rate,	this->local_update_rate_))	PARAM_WARN(p_up_rate,	up_rate_msg.str());
	if(!this->nh_.getParam(p_x_dim,	 this->local_x_size_))			PARAM_WARN(p_x_dim,		x_dim_msg.str());
	if(!this->nh_.getParam(p_y_dim,	 this->local_y_size_))			PARAM_WARN(p_y_dim,		y_dim_msg.str());
	if(!this->nh_.getParam(p_z_dim,	 this->local_z_size_))			PARAM_WARN(p_z_dim,		z_dim_msg.str());
	if(!this->nh_.getParam(p_x_ori,	 this->local_x_ori_))			PARAM_WARN(p_x_ori,		p_x_ori_msg.str());
	if(!this->nh_.getParam(p_y_ori,	 this->local_y_ori_))			PARAM_WARN(p_y_ori,		p_y_ori_msg.str());
	if(!this->nh_.getParam(p_z_ori,	 this->local_z_ori_))			PARAM_WARN(p_z_ori,		p_z_ori_msg.str());
	if(!this->nh_.getParam(p_res,	 this->local_res_))				PARAM_WARN(p_res,		p_res_msg.str());
	if(!this->nh_.getParam(pg_x_dim, this->global_x_size_))			PARAM_WARN(p_x_dim,		x_dim_msg.str());
	if(!this->nh_.getParam(pg_y_dim, this->global_y_size_))			PARAM_WARN(p_y_dim,		y_dim_msg.str());
	if(!this->nh_.getParam(pg_z_dim, this->global_z_size_))			PARAM_WARN(p_z_dim,		z_dim_msg.str());
	if(!this->nh_.getParam(pg_x_ori, this->global_x_ori_))			PARAM_WARN(p_x_ori,		p_x_ori_msg.str());
	if(!this->nh_.getParam(pg_y_ori, this->global_y_ori_))			PARAM_WARN(p_y_ori,		p_y_ori_msg.str());
	if(!this->nh_.getParam(pg_z_ori, this->global_z_ori_))			PARAM_WARN(p_z_ori,		p_z_ori_msg.str());
	if(!this->nh_.getParam(pg_res,	 this->global_res_))			PARAM_WARN(p_res,		p_res_msg.str());
}

void GlobalPlanner::registerTopics()
{
	//Comunication Parameters
	std::string local_planner_topic(OCCUPANCY_TOPIC);
	std::string odometry_topic(ODOMETRY_TOPIC);
	std::string command_topic("/global_planning/commands");


	this->laser_topic_           = "/laser";
	this->local_occupancy_topic_ = local_planner_topic;
	this->odom_topic_            = odometry_topic;

	//Get Private Parameters
	if(!this->p_nh_.getParam(local_planner_topic,this->local_occupancy_topic_))	PARAM_WARN(local_planner_topic,	local_planner_topic);
	if(!this->p_nh_.getParam(odometry_topic,	 this->odom_topic_))		    PARAM_WARN(odometry_topic,		odometry_topic);

	this->local_occ_pub_ = this->nh_.advertise<aero_path_planning::OccupancyGridMsg>(this->local_occupancy_topic_, 2);
	this->laser_sub_     = this->nh_.subscribe(this->laser_topic_, 2, &GlobalPlanner::laserCB, this);
	this->odom_sub_      = this->nh_.subscribe(this->odom_topic_,  2, &GlobalPlanner::odomCB,  this);
}

void GlobalPlanner::registerTimers()
{
	this->chunck_timer_ = this->nh_.createTimer(this->local_update_rate_, &GlobalPlanner::chunckCB, this);
}

void GlobalPlanner::laserCB(const sensor_msgs::PointCloud2ConstPtr& message)
{
	//ROS_INFO("Got a new Laser Scan!");
	pcl::PointCloud<pcl::PointXYZ> scan_cloud;
	Point origin;
	origin.x = 0;
	origin.y = this->local_y_size_/2;
	origin.z = 0;
	OccupancyGrid local_map(this->local_x_size_, this->local_y_size_, .01, origin, aero_path_planning::UNKNOWN);
	pcl::fromROSMsg(*message, scan_cloud);
	aero_path_planning::PointConverter converter(.1);

#pragma omp parallel for
	for(int i=0; i<(int)scan_cloud.size(); i++)
	{
		Point opoint;
		opoint.x = scan_cloud.at(i).x;
		opoint.y = scan_cloud.at(i).y;
		opoint.z = 0;
		converter.convertToGrid(opoint, opoint);

		Point on_grid;
		on_grid.getVector4fMap() = opoint.getVector4fMap()+origin.getVector4fMap();
		if(on_grid.x>=0&&on_grid.x<this->local_x_size_&&on_grid.y>0&&on_grid.y<this->local_y_size_)
		{
			local_map.setPointTrait(opoint, aero_path_planning::OBSTACLE);
		}
	}

	aero_path_planning::OccupancyGridMsg message_out;
	local_map.generateMessage(message_out);
	this->local_occ_pub_.publish(message_out);

}

bool GlobalPlanner::lidarToGlobal(const sensor_msgs::PointCloud2& scan_cloud, sensor_msgs::PointCloud2& result_cloud) const
{
	bool success = this->transformer_.canTransform(this->global_frame_, this->lidar_frame_, ros::Time::now());
	if(success)
	{
		return pcl_ros::transformPointCloud(this->global_frame_, scan_cloud, result_cloud, this->transformer_);
	}
	else
	{
		return success;
	}
}

void GlobalPlanner::lidarMsgToOccGridPatch(const sensor_msgs::PointCloud2& scan_cloud, aero_path_planning::PointCloud& result_cloud) const
{
	pcl::PointCloud<pcl::PointXYZ> copy_cloud;
	pcl::fromROSMsg(scan_cloud, copy_cloud);

	for(int i=0; i<(int)copy_cloud.size(); i++)
	{
		aero_path_planning::Point copy_point;
		copy_point.x    = copy_cloud.at(i).x;
		copy_point.y    = copy_cloud.at(i).y;
		copy_point.z    = copy_cloud.at(i).z;
		copy_point.rgba = aero_path_planning::OBSTACLE;
		result_cloud.push_back(copy_point);
	}
}

void GlobalPlanner::odomCB(const nav_msgs::OdometryConstPtr& message)
{
	this->last_odom_ = *message;
}

void GlobalPlanner::chunckCB(const ros::TimerEvent& event)
{

}
