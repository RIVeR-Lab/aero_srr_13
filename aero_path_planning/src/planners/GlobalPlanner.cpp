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
#include<geometry_msgs/PointStamped.h>
#include<occupancy_grid/MultiTraitOccupancyGridMessage.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/planners/GlobalPlanner.h>
#include<aero_path_planning/planning_strategies/RRTCarrot.h>
#include<aero_path_planning/OccupancyGridMsg.h>
#include<aero_srr_msgs/StateTransitionRequest.h>
//**********************NAMESPACES*****************************//


using namespace aero_path_planning;

GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh, app::CarrotPathFinder& path_planner):
																										path_planner_(&path_planner),
																										nh_(nh),
																										p_nh_(p_nh),
																										transformer_(nh)
{
	ROS_INFO("Initializing Global Planner...");
	this->state_.state = aero_srr_msgs::AeroState::SEARCH;
	this->loadOccupancyParam();
	this->registerTopics();
	this->registerTimers();
	//this->setManual(true);
	this->buildGlobalMap();

	this->cf_ = boost::bind(&GlobalPlanner::checkCollision, this, _1, _2);
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
	this->local_update_rate_ = 1.0/20.0;
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
	this->global_x_size_ = 5000;
	std::stringstream gx_dim_msg;
	gx_dim_msg<<this->global_x_size_<<"*0.05m";

	//y dimension of global occupancy grid
	std::string pg_y_dim(G_OCC_YDIM);
	this->global_y_size_ = 5000;
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
	this->global_x_ori_ = this->global_x_size_/2.0;
	std::stringstream pg_x_ori_msg;
	pg_x_ori_msg<<this->global_x_ori_<<"m";

	//y coord of the global origin of the occupancy grids
	std::string pg_y_ori(G_OCC_YORG);
	this->global_y_ori_ = this->global_y_size_/2.0;
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

	//The frame_id for the world occupancy grid
	std::string global_frame = "global_frame";
	this->global_frame_      = "/world";

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
	std::string lidar_topic(LIDAR_GLOBAL_TOPIC);
	std::string command_topic("/global_planning/commands");


	this->global_laser_topic_    = lidar_topic;
	this->local_occupancy_topic_ = local_planner_topic;

	//Get Private Parameters
	if(!this->p_nh_.getParam(local_planner_topic,this->local_occupancy_topic_))	PARAM_WARN(local_planner_topic,	local_planner_topic);
	if(!this->p_nh_.getParam(lidar_topic,	     this->global_laser_topic_))    PARAM_WARN(lidar_topic,		    lidar_topic);

	this->local_occ_pub_ = this->nh_.advertise<occupancy_grid::MultiTraitOccupancyGridMessage>(this->local_occupancy_topic_, 2);
	this->laser_sub_     = this->nh_.subscribe(this->global_laser_topic_, 2, &GlobalPlanner::laserCB, this);
	//this->map_viz_pub_   = this->nh_.advertise<aero_path_planning::OccupancyGridMsg>("aero/global/vizualization", 2, true);
	this->path_pub_      = this->nh_.advertise<nav_msgs::Path>("aero/global/path", 1, true);
	//this->slam_sub_      = this->nh_.subscribe("/map", 2, &GlobalPlanner::slamCB, this);
	this->state_sub      = this->nh_.subscribe("/aero/state", 1, &GlobalPlanner::stateCB, this);
	this->mission_goal_sub_ = this->nh_.subscribe("/aero/global/mission_goal", 1, &GlobalPlanner::missionGoalCB, this);
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
	nav_msgs::MapMetaData info;
	info.width = this->global_x_size_;
	info.height= this->global_y_size_;
	info.origin.position.x = -((double)this->global_x_ori_*this->global_res_)/2.0;
	info.origin.position.y = -((double)this->global_y_ori_*this->global_res_)/2.0;
	info.origin.position.z = 0;
	info.map_load_time = ros::Time::now();
	info.resolution    = this->global_res_;
	this->traits_.push_back(occupancy_grid::utilities::CellTrait::UNKOWN);
	this->traits_.push_back(occupancy_grid::utilities::CellTrait::FREE_LOW_COST);
	this->traits_.push_back(occupancy_grid::utilities::CellTrait::FREE_HIGH_COST);
	this->traits_.push_back(occupancy_grid::utilities::CellTrait::OBSTACLE);
	this->traits_.push_back(occupancy_grid::utilities::CellTrait::TRAVERSED);
	this->traits_.push_back(occupancy_grid::utilities::CellTrait::GOAL);
	this->global_map_ = occupancy_grid::MultiTraitOccupancyGridPtr(new occupancy_grid::MultiTraitOccupancyGrid(this->global_frame_, this->traits_, occupancy_grid::utilities::CellTrait::UNKOWN, info, info.width/2, info.height/2));


	this->local_info_.height = this->local_y_size_;
	this->local_info_.width  = this->local_x_size_;
	this->local_info_.resolution = this->local_res_;
	this->local_info_.origin.position.x = 0;
	this->local_info_.origin.position.y = -((double)(local_info_.height)*this->local_res_)/2.0;
}

void GlobalPlanner::laserCB(const sensor_msgs::PointCloud2ConstPtr& message)
{
	//ROS_INFO("Got a new Laser Scan!");
	/*	pcl::PointCloud<pcl::PointXYZ> scan_cloud;
	pcl::fromROSMsg(*message, scan_cloud);
	const PointConverter& converter = this->global_map_->getConverter();

#pragma omp parallel for
	for(int i=0; i<(int)scan_cloud.size(); i++)
	{
		Point opoint;
		opoint.x = scan_cloud.at(i).x;
		opoint.y = scan_cloud.at(i).y;
		opoint.z = 0;
		converter.convertToGrid(opoint, opoint);
		try
		{
			this->global_map_->setPointTrait(opoint, aero_path_planning::OBSTACLE);
		}
		catch(std::exception& e)
		{
			//do nothing, just means we got data past the edge of the global map
		}
	}
	 */
}

bool GlobalPlanner::lidarToGlobal(const sm::PointCloud2& scan_cloud, sm::PointCloud2& result_cloud) const
{
	this->transformer_.waitForTransform(this->global_frame_, scan_cloud.header.frame_id, scan_cloud.header.stamp, ros::Duration(this->local_update_rate_/2.0));
	return pcl_ros::transformPointCloud(this->global_frame_, scan_cloud, result_cloud, this->transformer_);
}

void GlobalPlanner::lidarMsgToOccGridPatch(const sm::PointCloud2& scan_cloud, app::PointCloud& result_cloud) const
{
	//	pcl::PointCloud<pcl::PointXYZ> copy_cloud;
	//	pcl::fromROSMsg(scan_cloud, copy_cloud);
	//
	//#pragma omp parallel for
	//	for(int i=0; i<(int)copy_cloud.size(); i++)
	//	{
	//		aero_path_planning::Point copy_point;
	//		copy_point.x    = copy_cloud.at(i).x;
	//		copy_point.y    = copy_cloud.at(i).y;
	//		copy_point.z    = copy_cloud.at(i).z;
	//		copy_point.rgba = aero_path_planning::OBSTACLE;
	//		result_cloud.push_back(copy_point);
	//	}
}

bool GlobalPlanner::calcRobotPointWorld(tf::Point& point) const
{
	bool success = true;
	geometry_msgs::PoseStamped robot_pose;
	robot_pose.header.frame_id    = this->local_frame_;
	robot_pose.header.stamp       = ros::Time::now();
	robot_pose.pose.orientation.w = 1;

	//look up the robot's position in the world frame
	try
	{
		this->transformer_.waitForTransform(this->global_frame_, robot_pose.header.frame_id, robot_pose.header.stamp, ros::Duration(0.1));
		this->transformer_.transformPose(this->global_frame_, robot_pose, robot_pose);
		tf::pointMsgToTF(robot_pose.pose.position, point);
	}
	catch(std::exception& e)
	{
		ROS_ERROR_STREAM_THROTTLE(1, e.what());
		success = false;
	}
	return success;
}

void GlobalPlanner::chunckCB(const ros::TimerEvent& event)
{
	//ROS_INFO_STREAM("I'm Chunking the Global Map!");
	occupancy_grid::MultiTraitOccupancyGridMessagePtr message(new occupancy_grid::MultiTraitOccupancyGridMessage);
	occupancy_grid::MultiTraitOccupancyGrid local_gird(this->local_frame_, this->traits_, occupancy_grid::utilities::CellTrait::UNKOWN, this->local_info_, 0, this->local_info_.height/2);
	local_gird.toROSMsg(*message);
	this->local_occ_pub_.publish(message);


}

void GlobalPlanner::slamCB(const nm::OccupancyGridConstPtr& message)
{
	ROS_INFO_STREAM("Recieved new SLAM map information!");
	//this->global_map_->setPointTrait(*message);
}


void GlobalPlanner::stateCB(const aero_srr_msgs::AeroStateConstPtr& message)
{
	typedef aero_srr_msgs::AeroState state_t;
	this->state_ = *message;
	geometry_msgs::Pose pose1;
	switch(message->state)
	{
	case state_t::ERROR:
	case state_t::MANUAL:
	case state_t::PAUSE:
	case state_t::SAFESTOP:
	case state_t::SHUTDOWN:
	case state_t::COLLECT:
		this->setManual(true);
		break;
	case state_t::SEARCH:
		this->setSearch();
		break;
	case state_t::NAVOBJ:
		this->setNavObj();
		break;
	default:
		ROS_ERROR_STREAM("Received Unkown State: "<<*message);
		break;
	}
}

void GlobalPlanner::planCB(const ros::TimerEvent& event)
{
	ROS_INFO_STREAM("Performing regularly scheduled global planning!");
	this->plan();
}

void GlobalPlanner::plan()
{
	ROS_INFO_STREAM("I'm making a new global plan using strategy "<<this->state_);
	if(this->path_planner_!=NULL&&this->global_map_!=occupancy_grid::MultiTraitOccupancyGridPtr())
	{
		std::deque<geometry_msgs::Pose> temp_path;
		this->path_planner_->setCollision(this->cf_);
		this->path_planner_->setCarrotDelta(5.0/this->global_res_);
		this->path_planner_->setSearchMap(this->global_map_);
		tf::Point current_point;
		if(this->calcRobotPointWorld(current_point))
		{
			tf::pointTFToMsg(current_point, this->current_point_.position);
			//Need to use a temporary path because carrot_path might be being used by other callbacks in multi-threaded spinner and this will lock it for an extended period
			this->path_planner_->search(this->current_point_, this->mission_goal_.pose, this->plan_timerout_, temp_path);
			//Copy the temp path over to replace the old path
			this->carrot_path_ = temp_path;
			//Remove the start location from the path as it's the current location at best and more likely well behind the robot at this point
			this->carrot_path_.pop_front();
			nav_msgs::PathPtr path(new nav_msgs::Path());
			path->header.frame_id = this->global_frame_;
			path->header.stamp    = ros::Time::now();
			this->carrotToPath(*path);
			this->path_pub_.publish(path);
			ROS_INFO_STREAM("I published a path of size: "<<path->poses.size());
		}
		else
		{
			ROS_ERROR("Could Not Plan Due to Inability to Look Up Robot in World!");
		}
	}
	else
	{
		ROS_ERROR("Cannot Make Global Plan Without a Planning Strategy!");
	}
}

void GlobalPlanner::carrotToPath(nav_msgs::Path& path) const
{
	BOOST_FOREACH(std::deque<geometry_msgs::Pose>::value_type point, this->carrot_path_)
					{
		geometry_msgs::PoseStamped path_pose;
		path_pose.header = path.header;
		path_pose.pose   = point;
		path.poses.push_back(path_pose);
					}
}

bool GlobalPlanner::checkCollision(const tf::Point& point, const occupancy_grid::MultiTraitOccupancyGrid& map) const
{
	bool collision = false;
	try
	{
		if(map.getPointTrait((int)point.x(), (int)point.y())==occupancy_grid::utilities::CellTrait::OBSTACLE)
		{
			collision = true;
		}
	}
	catch(bool& e)
	{
		collision = true;
	}
	return collision;
}

void GlobalPlanner::visualizeMap() const
{

}

void GlobalPlanner::setManual(bool enable)
{
	if(enable)
	{
		ROS_INFO_STREAM("Global Planner: I'm in Manual Mode!");
		this->plan_timer_.stop();
		this->chunck_timer_.stop();
	}
	else
	{
		ROS_INFO_STREAM("Global Planner: I'm in Autonomous Mode!");
		this->plan_timer_.start();
		this->chunck_timer_.start();
	}
}

void GlobalPlanner::setSearch()
{
	ROS_INFO_STREAM("Global Planner: I'm searching!");
	//TODO actually implement
	this->setManual(false);
}

void GlobalPlanner::setNavObj()
{
	ROS_INFO_STREAM("Global Planner: I'm Naving to Obj!");
	//TODO actually implement
	this->setManual(false);
}

void GlobalPlanner::missionGoalCB(const geometry_msgs::PoseStampedConstPtr& message)
{
	//Convert from the message's frame to the mapping frame (should be the same, but might not be)
	try
	{
		this->transformer_.waitForTransform(this->global_frame_, message->header.frame_id, message->header.stamp, ros::Duration(1.0));
		this->transformer_.transformPose(this->global_frame_, *message, this->mission_goal_);
		ROS_INFO_STREAM("Got New Mission Goal! Replanning...");
		this->plan();
	}
	catch(std::exception& e)
	{
		ROS_ERROR_STREAM_THROTTLE(1, e.what());
	}
}

