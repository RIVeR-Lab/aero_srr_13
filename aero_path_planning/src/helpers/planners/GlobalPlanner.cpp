/**
 * @file GlobalPlanner.cpp
 *
 * @date   Feb 19, 2013
 * @author Adam Panzica
 * @brief Implementaiton of GlobalPlanner
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<boost/foreach.hpp>
#include<pcl_ros/transforms.h>
#include<geometry_msgs/PoseStamped.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/planners/GlobalPlanner.h>
#include<aero_path_planning/planning_strategies/RRTCarrot.h>
#include<aero_path_planning/OccupancyGridMsg.h>
//**********************NAMESPACES*****************************//


using namespace aero_path_planning;

GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh, aero_path_planning::CarrotPathFinder& path_planner):
														state_(MANUAL),
														path_planner_(&path_planner),
														path_threshold_(1.0),
														nh_(nh),
														p_nh_(p_nh),
														transformer_(nh)
{
	ROS_INFO("Initializing Global Planner...");

	this->loadOccupancyParam();
	this->registerTopics();
	this->registerTimers();
	this->buildGlobalMap();

	this->cf_ = boost::bind(&GlobalPlanner::checkCollision, this, _1, _2);
	this->planCB(ros::TimerEvent());

	ROS_INFO("Global Planner Running!");

}

GlobalPlanner::~GlobalPlanner()
{

}

void GlobalPlanner::loadOccupancyParam()
{
	ROS_INFO_STREAM("Loading Global Configuration Params...");
	//Configuration Parameters
	//*****************Configuration Parameters*******************//
	//Update rate to generate local occupancy grid
	std::string p_up_rate(L_OCC_UPDTRT);
	this->local_update_rate_ = 1.0/40.0;
	std::stringstream up_rate_msg;
	up_rate_msg<<this->local_update_rate_<<"s";

	//x dimension of occupancy grid
	std::string p_x_dim(L_OCC_XDIM);
	this->local_x_size_ = 200;
	std::stringstream x_dim_msg;
	x_dim_msg<<this->local_x_size_<<"*0.05m";

	//y dimension of occupancy grid
	std::string p_y_dim(L_OCC_YDIM);
	this->local_y_size_ = 200;
	std::stringstream y_dim_msg;
	y_dim_msg<<this->local_y_size_<<"*0.05m";

	//z dimension of occupancy grid
	std::string p_z_dim(L_OCC_ZDIM);
	this->local_z_size_ = 0;
	std::stringstream z_dim_msg;
	z_dim_msg<<this->local_z_size_<<"m";

	//resolution occupancy grid
	std::string p_res(L_OCC_RES);
	this->local_res_ = .05;
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

	//Update rate to generate local occupancy grid
	std::string pg_up_rate(G_OCC_UPDTRT);
	this->global_update_rate_ = 30.0;
	std::stringstream gup_rate_msg;
	gup_rate_msg<<this->global_update_rate_<<"s";

	//x dimension of global occupancy grid
	std::string pg_x_dim(G_OCC_XDIM);
	this->global_x_size_ = 1000;
	std::stringstream gx_dim_msg;
	gx_dim_msg<<this->global_x_size_<<"*0.05m";

	//y dimension of global occupancy grid
	std::string pg_y_dim(G_OCC_YDIM);
	this->global_y_size_ = 1000;
	std::stringstream gy_dim_msg;
	gy_dim_msg<<this->global_y_size_<<"*0.05m";

	//z dimension of global occupancy grid
	std::string pg_z_dim(G_OCC_ZDIM);
	this->global_z_size_ = 0;
	std::stringstream gz_dim_msg;
	gz_dim_msg<<this->global_z_size_<<"*0.05m";

	//resolution global occupancy grid
	std::string pg_res(G_OCC_RES);
	this->global_res_ = .05;
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

	//The frame_id for the local occupancy grid
	std::string local_frame = "local_frame";
	this->local_frame_      = "/base_footprint";

	//Get Public Parameters
	if(!this->nh_.getParam(p_up_rate,	this->local_update_rate_))	PARAM_WARN(p_up_rate,	up_rate_msg.str());
	if(!this->nh_.getParam(pg_up_rate,	this->global_update_rate_))	PARAM_WARN(pg_up_rate,	gup_rate_msg.str());
	if(!this->nh_.getParam(p_x_dim,	 this->local_x_size_))			PARAM_WARN(p_x_dim,		x_dim_msg.str());
	if(!this->nh_.getParam(p_y_dim,	 this->local_y_size_))			PARAM_WARN(p_y_dim,		y_dim_msg.str());
	if(!this->nh_.getParam(p_z_dim,	 this->local_z_size_))			PARAM_WARN(p_z_dim,		z_dim_msg.str());
	if(!this->nh_.getParam(p_x_ori,	 this->local_x_ori_))			PARAM_WARN(p_x_ori,		p_x_ori_msg.str());
	if(!this->nh_.getParam(p_y_ori,	 this->local_y_ori_))			PARAM_WARN(p_y_ori,		p_y_ori_msg.str());
	if(!this->nh_.getParam(p_z_ori,	 this->local_z_ori_))			PARAM_WARN(p_z_ori,		p_z_ori_msg.str());
	if(!this->nh_.getParam(p_res,	 this->local_res_))				PARAM_WARN(p_res,		p_res_msg.str());
	if(!this->nh_.getParam(pg_x_dim, this->global_x_size_))			PARAM_WARN(pg_x_dim,	gx_dim_msg.str());
	if(!this->nh_.getParam(pg_y_dim, this->global_y_size_))			PARAM_WARN(pg_y_dim,	gy_dim_msg.str());
	if(!this->nh_.getParam(pg_z_dim, this->global_z_size_))			PARAM_WARN(pg_z_dim,	gz_dim_msg.str());
	if(!this->nh_.getParam(pg_x_ori, this->global_x_ori_))			PARAM_WARN(pg_x_ori,	pg_x_ori_msg.str());
	if(!this->nh_.getParam(pg_y_ori, this->global_y_ori_))			PARAM_WARN(pg_y_ori,	pg_y_ori_msg.str());
	if(!this->nh_.getParam(pg_z_ori, this->global_z_ori_))			PARAM_WARN(pg_z_ori,    pg_z_ori_msg.str());
	if(!this->nh_.getParam(pg_res,	 this->global_res_))			PARAM_WARN(pg_res,		pg_res_msg.str());
}

void GlobalPlanner::registerTopics()
{
	ROS_INFO_STREAM("Registering Global Topics...");
	//Comunication Parameters
	std::string local_planner_topic(OCCUPANCY_TOPIC);
	std::string odometry_topic(ODOMETRY_TOPIC);
	std::string lidar_topic(LIDAR_GLOBAL_TOPIC);
	std::string command_topic("/global_planning/commands");


	this->global_laser_topic_    = lidar_topic;
	this->local_occupancy_topic_ = local_planner_topic;
	this->odom_topic_            = odometry_topic;

	//Get Private Parameters
	if(!this->p_nh_.getParam(local_planner_topic,this->local_occupancy_topic_))	PARAM_WARN(local_planner_topic,	local_planner_topic);
	if(!this->p_nh_.getParam(odometry_topic,	 this->odom_topic_))		    PARAM_WARN(odometry_topic,		odometry_topic);
	if(!this->p_nh_.getParam(lidar_topic,	     this->global_laser_topic_))    PARAM_WARN(lidar_topic,		    lidar_topic);

	this->local_occ_pub_ = this->nh_.advertise<aero_path_planning::OccupancyGridMsg>(this->local_occupancy_topic_, 2);
	this->laser_sub_     = this->nh_.subscribe(this->global_laser_topic_, 2, &GlobalPlanner::laserCB, this);
	this->odom_sub_      = this->nh_.subscribe(this->odom_topic_,  2, &GlobalPlanner::odomCB,  this);
}

void GlobalPlanner::registerTimers()
{
	ROS_INFO_STREAM("Registering Global Timers...");
	this->chunck_timer_ = this->nh_.createTimer(ros::Duration(this->local_update_rate_),  &GlobalPlanner::chunckCB, this);
	this->plan_timer_   = this->nh_.createTimer(ros::Duration(this->global_update_rate_), &GlobalPlanner::planCB,   this);
}

void GlobalPlanner::buildGlobalMap()
{
	ROS_INFO_STREAM("Building initial global map!");
	Point origin;
	origin.x = this->global_x_ori_;
	origin.y = this->global_y_ori_;
	origin.z = this->global_z_ori_;
	origin.rgba = aero_path_planning::UNKNOWN;
	this->global_map_ = OccupancyGridPtr(new OccupancyGrid(this->global_x_size_, this->global_y_size_, this->global_z_size_, this->global_res_, origin, aero_path_planning::UNKNOWN, this->global_frame_));
}

void GlobalPlanner::laserCB(const sensor_msgs::PointCloud2ConstPtr& message)
{
	//ROS_INFO("Got a new Laser Scan!");
	pcl::PointCloud<pcl::PointXYZ> scan_cloud;
	Point origin;
	origin.x = 0;
	origin.y = this->local_y_size_/2;
	origin.z = 0;
	OccupancyGrid local_map(this->local_x_size_, this->local_y_size_, this->local_res_, origin, aero_path_planning::UNKNOWN);
	pcl::fromROSMsg(*message, scan_cloud);
	aero_path_planning::PointConverter converter(this->local_res_);

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

	OccupancyGridMsgPtr message_out(new OccupancyGridMsg());
	local_map.generateMessage(*message_out);
	this->local_occ_pub_.publish(message_out);

}

bool GlobalPlanner::lidarToGlobal(const sensor_msgs::PointCloud2& scan_cloud, sensor_msgs::PointCloud2& result_cloud) const
{
	this->transformer_.waitForTransform(this->global_frame_, scan_cloud.header.frame_id, scan_cloud.header.stamp, ros::Duration(this->local_update_rate_/2.0));
	return pcl_ros::transformPointCloud(this->global_frame_, scan_cloud, result_cloud, this->transformer_);
}

void GlobalPlanner::lidarMsgToOccGridPatch(const sensor_msgs::PointCloud2& scan_cloud, aero_path_planning::PointCloud& result_cloud) const
{
	pcl::PointCloud<pcl::PointXYZ> copy_cloud;
	pcl::fromROSMsg(scan_cloud, copy_cloud);

#pragma omp parallel for
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
	ROS_INFO_STREAM("I Got New Odometry Data!");
	this->last_odom_ = *message;
	if(!this->carrot_path_.empty())
	{
		geometry_msgs::PointStamped trans_point_m;
		geometry_msgs::PointStamped odom_point_m;
		odom_point_m.header = message->header;
		odom_point_m.point  = message->pose.pose.position;

		//Shift the robot location from the odometry frame to the global one
		this->transformer_.transformPoint(this->global_frame_, odom_point_m, trans_point_m);

		//Check the distance between the current robot location and the next path goal point.
		//If within threshold, pop the path goal point
		Point current_point;
		current_point.x = trans_point_m.point.x;
		current_point.y = trans_point_m.point.y;
		current_point.z = trans_point_m.point.z;

		double dist = pcl::distances::l2(current_point.getVector4fMap(), this->carrot_path_.front().getVector4fMap());
		if(std::abs(dist)<this->path_threshold_)
		{
			this->carrot_path_.pop();
		}
	}
}

void GlobalPlanner::chunckCB(const ros::TimerEvent& event)
{
	//ROS_INFO_STREAM("I'm Chunking the Global Map!");
	Point origin;
	origin.x = this->local_x_ori_;
	origin.y = this->local_y_ori_;
	origin.z = this->local_z_ori_;
	origin.rgb = aero_path_planning::ROBOT;
	OccupancyGrid local_grid(this->local_x_size_, this->local_y_size_, this->local_z_size_, this->local_res_, origin, aero_path_planning::UNKNOWN, this->local_frame_);

	OccupancyGridCloud copyCloud;
	//Transform the coordinates of the local grid to the global frame
	this->transformer_.waitForTransform(this->global_frame_, local_grid.getFrameId(), ros::Time::now(), ros::Duration(this->local_update_rate_/4.0));
	pcl_ros::transformPointCloud(this->global_frame_, local_grid.getGrid(), copyCloud, this->transformer_);
	//Copy the data in the global frame at the transformed local-coordinates into the local grid
#pragma omp parallel for
	for(int i=0; i<(int)copyCloud.size(); i++)
	{
		try
		{
			//Get rid of any rounding issues
			copyCloud.at(i).x = std::floor(copyCloud.at(i).x);
			copyCloud.at(i).y = std::floor(copyCloud.at(i).y);
			copyCloud.at(i).z = std::floor(copyCloud.at(i).z);

			//Copy the PointTrait data from the global frame to the local frame
			local_grid.setPointTrait(local_grid.getGrid().at(i), this->global_map_->getPointTrait(copyCloud.at(i)));
		}
		catch(std::runtime_error& e)
		{
			//Do nothing, means the local grid has gone outside the bounds of the global frame so we have no data anyway
		}
	}
	//Copy the current goal point to the occupancy grid
	this->copyNextGoalToGrid(local_grid);


	//Send the new local grid to the local planner
	OccupancyGridMsgPtr occ_grid_msg(new OccupancyGridMsg());
	local_grid.generateMessage(*occ_grid_msg);
	this->local_occ_pub_.publish(occ_grid_msg);
}

void GlobalPlanner::copyNextGoalToGrid(aero_path_planning::OccupancyGrid& grid) const
{
	//ROS_INFO_STREAM("I'm Copying the Next Carrot Path Point Onto the Local Grid in frame "<<grid.getFrameId());
	if(!this->carrot_path_.empty())
	{
		geometry_msgs::PointStamped goal_point_m;
		goal_point_m.point.x = this->carrot_path_.front().x;
		goal_point_m.point.x = this->carrot_path_.front().y;
		goal_point_m.point.x = this->carrot_path_.front().z;
		goal_point_m.header.frame_id = this->global_frame_;
		goal_point_m.header.stamp    = ros::Time::now();
		this->transformer_.transformPoint(grid.getFrameId(), goal_point_m, goal_point_m);
		Point goal_point;
		goal_point.x = goal_point_m.point.x;
		goal_point.y = goal_point_m.point.y;
		goal_point.z = goal_point_m.point.z;

		try
		{
			grid.setGoalPoint(goal_point);
		}
		catch(std::runtime_error& e)
		{
			ROS_ERROR_STREAM("Error Copying Next Carrot Path Point to Local Grid!:"<<e.what());
		}
	}
	else
	{
		ROS_WARN("I don't have a path to follow currently!");
	}
}

void GlobalPlanner::planCB(const ros::TimerEvent& event)
{
	ROS_INFO_STREAM("I'm making a new global plan using strategy "<<this->state_);
	this->carrot_path_ = std::queue<Point>();
	Point start_point;
	start_point.x = 0;
	start_point.y = 0;
	start_point.z = 0;
	Point goal_point;
	goal_point.x  = 100;
	goal_point.y  = 100;
	goal_point.z  = 0;
	if(this->path_planner_!=NULL)
	{
		this->path_planner_->setCollision(this->cf_);
		this->path_planner_->setCarrotDelta(10);
		this->path_planner_->setSearchMap(*this->global_map_);
		this->path_planner_->search(start_point, goal_point, this->plan_timerout_, this->carrot_path_);
	}
	else
	{
		ROS_ERROR("Cannot Make Global Plan Without a Planning Strategy!");
	}
}

bool GlobalPlanner::checkCollision(const aero_path_planning::Point& point, const aero_path_planning::OccupancyGrid& map) const
{
	bool collision = false;
	try
	{
		if(map.getPointTrait(point)==aero_path_planning::OBSTACLE)
		{
			collision = true;
		}
	}
	catch(std::exception& e)
	{
		collision = true;
	}
	return collision;
}
