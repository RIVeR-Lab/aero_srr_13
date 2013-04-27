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
	ROS_INFO_STREAM("Building Test Mission Goals...");
	geometry_msgs::Pose pose1;
	pose1.position.x = 10;
	pose1.position.y = 0;
	pose1.orientation.w = 1;
	this->mission_goals_.push_back(pose1);
	geometry_msgs::Pose pose2;
	pose2.position.x = 10;
	pose2.position.y = 10;
	pose2.orientation.w = 1;
	this->mission_goals_.push_back(pose2);
	geometry_msgs::Pose pose3;
	pose3.position.x = 0;
	pose3.position.y = 10;
	pose3.orientation.w = 1;
	this->mission_goals_.push_back(pose3);
	geometry_msgs::Pose pose4;
	pose4.position.x = 0;
	pose4.position.y = 0;
	pose4.orientation.w = 1;
	this->mission_goals_.push_back(pose4);
	ROS_INFO_STREAM("Test Mission Goals Built");

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
	this->odom_sub_      = this->nh_.subscribe("aero_base_pose_ekf/odom",  2, &GlobalPlanner::odomCB,  this);
	this->map_viz_pub_   = this->nh_.advertise<aero_path_planning::OccupancyGridMsg>("aero/global/vizualization", 2, true);
	this->goal_pub_      = this->nh_.advertise<geometry_msgs::PoseStamped>("/aero/global/goal", 2, true);
	this->path_pub_      = this->nh_.advertise<nav_msgs::Path>("aero/global/path", 2, true);
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

void GlobalPlanner::odomCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& message)
{
	ROS_INFO_STREAM("I Got New Odometry Data!");
	if(!this->carrot_path_.empty())
	{
		geometry_msgs::PointStamped trans_point_m;
		geometry_msgs::PointStamped odom_point_m;
		odom_point_m.header = message->header;
		odom_point_m.point  = message->pose.pose.position;

		//Shift the robot location from the odometry frame to the global one
		try
		{
			this->transformer_.waitForTransform(this->global_frame_, odom_point_m.header.frame_id, odom_point_m.header.stamp, ros::Duration(0.1));
			this->transformer_.transformPoint(this->global_frame_, odom_point_m, trans_point_m);
		}
		catch(std::exception& e)
		{
			ROS_ERROR_STREAM_THROTTLE(1, e.what());
		}

		//Check the distance between the current robot location and the next path goal point.
		//If within threshold, pop the path goal point
		Point current_point;
		current_point.x = trans_point_m.point.x;
		current_point.y = trans_point_m.point.y;
		current_point.z = 0;
		this->global_map_->getConverter().convertToGrid(current_point, this->current_point_);
		double dist = pcl::distances::l2(this->current_point_.getVector4fMap(), this->carrot_path_.front().getVector4fMap());
		ROS_INFO_STREAM_THROTTLE(1, "At position <"<<this->current_point_.x<<","<<this->current_point_.y<<">, Goal Position <"<<carrot_path_.front().x<<","<<carrot_path_.front().y<<">, dist="<<dist);
		if(std::abs(dist)<this->path_threshold_)
		{
			this->carrot_path_.pop_front();
			this->updateGoal();
		}
	}
	else
	{
		ROS_INFO_STREAM("Reached a Mission Goal, Moving to the next one!");
		if(!this->mission_goals_.empty())
		{
			this->mission_goals_.pop_front();
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
	try
	{
		//Transform the coordinates of the local grid to the global frame
		ros::Time transform_time = ros::Time::now();
		this->transformer_.waitForTransform(this->global_frame_, local_grid.getFrameId(), transform_time, ros::Duration(this->local_update_rate_));
		pcl_ros::transformPointCloud(this->global_frame_, transform_time, local_grid.getGrid(), local_grid.getFrameId(), copyCloud, this->transformer_);

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


		//Send the new local grid to the local planner
		OccupancyGridMsgPtr occ_grid_msg(new OccupancyGridMsg());
		local_grid.generateMessage(*occ_grid_msg);
		this->local_occ_pub_.publish(occ_grid_msg);
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM_THROTTLE(1, e.what());
	}
}

void GlobalPlanner::updateGoal() const
{
	//ROS_INFO_STREAM("I'm Copying the Next Carrot Path Point Onto the Local Grid in frame "<<grid.getFrameId());
	if(!this->carrot_path_.empty())
	{
		Point goal_point;
		this->global_map_->getConverter().convertToEng(this->carrot_path_.front(), goal_point);
		geometry_msgs::PointStamped goal_point_m;
		goal_point_m.point.x = goal_point.x;
		goal_point_m.point.y = goal_point.y;
		goal_point_m.point.z = 0;
		goal_point_m.header.frame_id = this->global_frame_;
		goal_point_m.header.stamp    = ros::Time::now();
		geometry_msgs::PoseStamped goal_pose;
		goal_pose.header = goal_point_m.header;
		goal_pose.pose.position    = goal_point_m.point;
		goal_pose.pose.orientation.w = 1;
		this->goal_pub_.publish(goal_pose);
	}

}

void GlobalPlanner::planCB(const ros::TimerEvent& event)
{
	ROS_INFO_STREAM("I'm making a new global plan using strategy "<<this->state_);
	this->carrot_path_ = std::deque<Point>();
	if(!this->mission_goals_.empty())
	{
		Point goal_point;
		goal_point.x = this->mission_goals_.front().position.x;
		goal_point.y = this->mission_goals_.front().position.y;
		goal_point.z = 0;
		this->global_map_->getConverter().convertToGrid(goal_point, goal_point);
		if(this->path_planner_!=NULL)
		{
			this->path_planner_->setCollision(this->cf_);
			this->path_planner_->setCarrotDelta(5.0/this->global_res_);
			this->path_planner_->setSearchMap(*this->global_map_);
			this->path_planner_->search(this->current_point_, goal_point, this->plan_timerout_, this->carrot_path_);
			this->carrot_path_.pop_front();
			nav_msgs::PathPtr path(new nav_msgs::Path());
			path->header.frame_id = this->global_frame_;
			path->header.stamp    = ros::Time::now();
			this->carrotToPath(*path);
			this->path_pub_.publish(path);
			this->updateGoal();
		}
		else
		{
			ROS_ERROR("Cannot Make Global Plan Without a Planning Strategy!");
		}
	}
	else
	{
		ROS_WARN("No Mission Goals!");
	}

	OccupancyGridMsgPtr viz_message(new OccupancyGridMsg());
	this->global_map_->generateMessage(*viz_message);
	this->map_viz_pub_.publish(viz_message);
}

void GlobalPlanner::carrotToPath(nav_msgs::Path& path) const
{
	Point path_point;
	const PointConverter& converter = this->global_map_->getConverter();
	BOOST_FOREACH(std::deque<Point>::value_type point, this->carrot_path_)
	{
		converter.convertToEng(point, path_point);
		geometry_msgs::PoseStamped path_pose;
		path_pose.header             = path.header;
		path_pose.pose.position.x    = path_point.x;
		path_pose.pose.position.y    = path_point.y;
		path_pose.pose.position.z    = path_point.z;
		path_pose.pose.orientation.w = 1;
		path.poses.push_back(path_pose);
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

void GlobalPlanner::visualizeMap() const
{

}
