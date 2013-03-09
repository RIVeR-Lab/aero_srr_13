/**
 * @file LocalPlanner.cpp
 *
 * @date Mar 9, 2013
 * @author Adam Panzica
 */

//********************** SYSTEM DEPENDANCIES **********************//

//********************** LOCAL  DEPENDANCIES **********************//
#include <aero_path_planning/LocalPlanner.h>
#include <aero_path_planning/FitnessQueue.h>
#include "OryxPathPlannerConfig.h"

using namespace aero_path_planning;



LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh) throw(std::runtime_error):
																		nh_(nh),
																		p_nh_(p_nh),
																		occupancy_buffer_(2)
{
	ROS_INFO("Starting Up Aero Local Planner Version %d.%d.%d", oryx_path_planner_VERSION_MAJOR, oryx_path_planner_VERSION_MINOR, oryx_path_planner_VERSION_BUILD);

	this->loadParam();
	this->regTopic();
	this->regTimers();

	this->should_plan_ = true;
	this->current_rad_ = 0;
	this->current_vel_ = 0;



	std::string software_stop_topic("aero/software_stop");

	if(!this->nh_.getParam(software_stop_topic, software_stop_topic))ROS_WARN("%s not set, using default value %s", software_stop_topic.c_str(), software_stop_topic.c_str());
	this->stop_sub_ = nh_.subscribe(software_stop_topic, 1, &LocalPlanner::stopCB, this);

	switch(this->platform_)
	{
	case 0:
		ROS_FATAL("Platofrm Oryx is no longer supported");
		break;
	case 1:
		ROS_INFO_STREAM("Publishing Twists to <"<<this->v_action_topic_<<">");
		break;
	default:
		ROS_FATAL("Unkown Platform, Could Not Setup Local Planner");
		break;

	}
	ROS_INFO("Aero Local Planner Running");
}

void LocalPlanner::loadParam()
{
	//Default Parameter Values
	//*****************Communication Parameters*******************//
	this->v_action_topic_ = VEL_CMD_TOPIC;
	//std::string t_com_top("translate_command_topic");
	this->pc_topic_       = OCCUPANCY_TOPIC;

	//*****************Configuration Parameters*******************//
	//The platform that the local planner is running on
	std::string p_platform(PLATFORM);
	this->platform_ = 0;
	std::string platform_message("Oryx");

	//*****************Configuration Parameters*******************//
	//Minimum update rate expected of occupancy grid
	std::string p_up_rate(L_OCC_UPDTRT);
	double update_rate = 0.2;
	std::string up_rate_msg("");
	up_rate_msg+= boost::lexical_cast<double>(update_rate);
	up_rate_msg+="s";

	//x dimension of occupancy grid
	std::string p_x_dim(L_OCC_XDIM);
	this->x_dim_ = 200;
	std::string x_dim_msg("");
	x_dim_msg+= boost::lexical_cast<double>(this->x_dim_);
	x_dim_msg+="m";

	//y dimension of occupancy grid
	std::string p_y_dim(L_OCC_YDIM);
	this->y_dim_ = 200;
	std::string y_dim_msg("");
	y_dim_msg+= boost::lexical_cast<double>(this->y_dim_);
	y_dim_msg+="m";

	//z dimension of occupancy grid
	std::string p_z_dim(L_OCC_ZDIM);
	this->z_dim_ = 0;
	std::string z_dim_msg("");
	z_dim_msg+= boost::lexical_cast<double>(this->z_dim_);
	z_dim_msg+="m";

	//resolution occupancy grid
	std::string p_res(L_OCC_RES);
	this->res_ = .01;
	std::string p_res_msg("");
	p_res_msg+= boost::lexical_cast<double>(this->res_);
	p_res_msg+="m";

	//x coord of the origin of the occupancy grids
	std::string p_x_ori(L_OCC_XORG);
	double x_ori = 0;
	std::string p_x_ori_msg("");
	p_x_ori_msg+= boost::lexical_cast<double>(x_ori);
	p_x_ori_msg+="m";

	//z coord of the origin of the occupancy grids
	std::string p_z_ori(L_OCC_YORG);
	double z_ori = 0;
	std::string p_z_ori_msg("");
	p_z_ori_msg+= boost::lexical_cast<double>(z_ori);
	p_z_ori_msg+="m";

	//y coord of the origin of the occupancy grids
	std::string p_y_ori(L_OCC_ZORG);
	double y_ori = this->y_dim_/2;
	std::string p_y_ori_msg("");
	p_y_ori_msg+= boost::lexical_cast<double>(y_ori);
	p_y_ori_msg+="m";


	//number of tentacles per speed set
	std::string p_num_tent(T_NUMBER);
	int num_tent = 81;
	std::string p_num_tent_msg("");
	p_num_tent_msg+= boost::lexical_cast<double>(num_tent);
	p_num_tent_msg+=" Tentacles";

	//Exponential Factor to use for generating seed radii
	std::string p_exp_fact(T_EXPFACT);
	double exp_fact = 1.15;
	std::string p_exp_fact_msg("");
	p_exp_fact_msg+= boost::lexical_cast<double>(exp_fact);

	//number of tentacles per speed set
	std::string p_num_speed_set(S_NUMBER);
	int num_speed_set = 15;
	std::string p_numSpeedSet_msg("");
	p_numSpeedSet_msg+= boost::lexical_cast<double>(num_speed_set);
	p_numSpeedSet_msg+= " Speed Sets";

	//Max Speed
	std::string p_max_speed(S_MAX_SPEED);
	double max_speed = 1;
	std::string p_max_speed_msg("");
	p_max_speed_msg+= boost::lexical_cast<double>(max_speed);
	p_max_speed_msg+="m/s";

	//Min Speed
	std::string p_min_speed(S_MIN_SPEED);
	double min_speed = 1;
	std::string p_min_speed_msg("");
	p_min_speed_msg+= boost::lexical_cast<double>(min_speed);
	p_min_speed_msg+="m/s";

	//Goal Weight
	std::string p_goal_weight(GOAL_WEIGHT);
	double goal_weight = 2;
	std::string p_goal_weight_msg("");
	p_goal_weight_msg+= boost::lexical_cast<double>(goal_weight);

	//Traversed Weight
	std::string p_trav_weight(TRAV_WEIGHT);
	double trav_weight = 0.1;
	std::string p_trav_weight_msg("");
	p_trav_weight_msg+= boost::lexical_cast<double>(trav_weight);

	//Difficulty Weight
	std::string p_diff_weight(DIFF_WEIGHT);
	double diff_weight = 0.1;
	std::string p_diff_weight_msg("");
	p_diff_weight_msg+= boost::lexical_cast<double>(diff_weight);

	//Unkown Terrain Weight
	std::string p_unkn_weight(UNKN_WEIGHT);
	double unkn_weight = 0.1;
	std::string p_unkn_weight_msg("");
	p_unkn_weight_msg+= boost::lexical_cast<double>(unkn_weight);

	//Get Private Parameters
	if(!p_nh_.getParam(p_platform, this->platform_))
	{
		PARAM_WARN(p_platform, platform_message);
	}
	else
	{
		switch(this->platform_)
		{
		case 0:
			platform_message = "Oryx";
			break;
		case 1:
			platform_message = "Husky A200";
			break;
		default:
			platform_message = "Unkown, defaulting to Oryx";
			break;
		}
		ROS_INFO_STREAM("Running on Platform: "<<platform_message);
	}
	if(!p_nh_.getParam(this->v_action_topic_,  this->v_action_topic_))	PARAM_WARN(this->v_action_topic_,  this->v_action_topic_);
	if(!p_nh_.getParam(this->pc_topic_,	       this->pc_topic_))	    PARAM_WARN(this->pc_topic_,	       this->pc_topic_);

	if(!p_nh_.getParam(p_goal_weight,	this->goal_weight_))    PARAM_WARN(p_goal_weight, p_goal_weight_msg);
	if(!p_nh_.getParam(p_trav_weight,	this->trav_weight_))    PARAM_WARN(p_trav_weight, p_trav_weight_msg);
	if(!p_nh_.getParam(p_diff_weight,	this->diff_weight_))    PARAM_WARN(p_diff_weight, p_diff_weight_msg);
	if(!p_nh_.getParam(p_unkn_weight,	this->unkn_weight_))    PARAM_WARN(p_unkn_weight, p_unkn_weight_msg);
	//Get Public Parameters
	if(!nh_.getParam(p_up_rate,	update_rate))	     PARAM_WARN(p_up_rate,	up_rate_msg);
	if(!nh_.getParam(p_x_dim,	this->x_dim_))		 PARAM_WARN(p_x_dim,	x_dim_msg);
	if(!nh_.getParam(p_y_dim,	this->y_dim_))		 PARAM_WARN(p_y_dim,	y_dim_msg);
	if(!nh_.getParam(p_z_dim,	this->z_dim_))		 PARAM_WARN(p_z_dim,	z_dim_msg);
	if(!nh_.getParam(p_x_ori,	x_ori))			     PARAM_WARN(p_x_ori,	p_x_ori_msg);
	if(!nh_.getParam(p_y_ori,	y_ori))			     PARAM_WARN(p_y_ori,	p_y_ori_msg);
	if(!nh_.getParam(p_z_ori,	z_ori))			     PARAM_WARN(p_z_ori,	p_z_ori_msg);
	if(!nh_.getParam(p_res,		this->res_))		 PARAM_WARN(p_res,		p_res_msg);
	if(!nh_.getParam(p_num_tent,	num_tent))		 PARAM_WARN(p_num_tent,	p_num_tent_msg);
	if(!nh_.getParam(p_exp_fact,	exp_fact))		 PARAM_WARN(p_exp_fact,	p_exp_fact_msg);
	if(!nh_.getParam(p_num_speed_set, num_speed_set))PARAM_WARN(p_num_speed_set,	p_numSpeedSet_msg);
	if(!nh_.getParam(p_max_speed,	max_speed))		 PARAM_WARN(p_max_speed,	    p_max_speed_msg);
	if(!nh_.getParam(p_min_speed,	min_speed))		 PARAM_WARN(p_min_speed,	    p_min_speed_msg);
	this->origin_.x = x_ori;
	this->origin_.y = y_ori;
	this->origin_.z = z_ori;
	this->tentacles_ = TentacleGeneratorPtr(new TentacleGenerator(min_speed,max_speed,num_speed_set, num_tent, exp_fact, this->res_, this->x_dim_, this->y_dim_));
}

void LocalPlanner::regTopic()
{
	this->pc_sub_  = this->nh_.subscribe(this->pc_topic_, 1, &LocalPlanner::pcCB, this);
	this->vel_pub_ = this->nh_.advertise<geometry_msgs::Twist>(this->v_action_topic_, 2);
	this->tent_pub_= this->nh_.advertise<sensor_msgs::PointCloud2>("/aero/tencale_visualization", 2);
}

void LocalPlanner::regTimers()
{
	this->vel_timer_ = nh_.createTimer(ros::Duration(1/20), &LocalPlanner::velUpdateCB, this);
	this->plan_timer_= nh_.createTimer(ros::Duration(1/5), &LocalPlanner::planningCB, this);
}

LocalPlanner::~LocalPlanner(){};

bool LocalPlanner::selectTentacle(const double& current_vel, const OccupancyGrid& search_grid, int& speedset_idx, int& tentacle_idx)
{
	FitnessQueue<double, boost::shared_ptr<std::pair<int, int> > > best_tentacle_candidates;
	SpeedSet slow_set    = this->tentacles_->getSpeedSet(1);
	SpeedSet current_set = this->tentacles_->getSpeedSet(2);
	SpeedSet fast_set    = this->tentacles_->getSpeedSet(3);
	bool   has_goal      = true;
	bool   hit_goal      = false;
//#pragma omp parallel for
	for(int i=0; i<(int)current_set.getNumTentacle(); i++)
	{
		//If we already hit the goal, short circuit since we can't break from OpenMP loops
		if(!hit_goal)
		{
			double length_modifier = 0;
			bool traversing = true;
			Tentacle working_tentacle = current_set.getTentacle(i);
			Tentacle::TentacleTraverser traverser(working_tentacle);
			length_modifier = 0;

			//As long as the tentacle has points still, and we're still traversing, continue traversing
			while(traverser.hasNext()&&traversing)
			{
				const aero_path_planning::Point& point = traverser.next();
				try
				{
					switch(search_grid.getPointTrait(point))
					{
					case aero_path_planning::OBSTACLE:
						//ROS_INFO("Hit Obstacle On Tentacle %d at length %f", i, traverser.lengthTraversed());
						//PRINT_POINT("Hit Point", point);
						traversing = false;
						break;
					case aero_path_planning::GOAL:
						//ROS_INFO("Hit the Goal on Tentacle %d at length %f", i, traverser.lengthTraversed());
						traversing = false;
						hit_goal   = true;
						break;
					case aero_path_planning::FREE_HIGH_COST:
						length_modifier -= traverser.deltaLength()*this->diff_weight_;
						break;
					case aero_path_planning::TRAVERSED:
						length_modifier -= traverser.deltaLength()*this->trav_weight_;
						break;
					case aero_path_planning::UNKNOWN:
						length_modifier += traverser.deltaLength()*this->unkn_weight_;
						break;
					default:
						break;
					}
				}catch(OccupancyGridAccessException& e)
				{
					ROS_ERROR("%s", e.what());
				}
			}
			//ROS_INFO_STREAM("I'm Checking The Distance To Goal");
			//Modify length based on closeness to goal, if there is one
			if(has_goal)
			{
				//ROS_INFO_STREAM("There is a goal...");
				try
				{
					//Will throw false if there was no goal point
					const aero_path_planning::Point goal_point = search_grid.getGoalPoint();
					const aero_path_planning::Point end_point = traverser.next();
					double dist_to_goal = pcl::distances::l2(end_point.getVector4fMap(), goal_point.getVector4fMap());
					//ROS_INFO_STREAM("Distance To Goal: "<<dist_to_goal);
					length_modifier-= dist_to_goal*this->goal_weight_;
				}
				catch(bool& e)
				{
					//ROS_INFO("There is not a goal");
					//There was no goal, set the flag
					has_goal = e;
				}
			}


			double tent_fitness = traverser.lengthTraversed()+length_modifier;
			boost::shared_ptr< std::pair<int, int> > tent_details(new std::pair<int, int>(current_set.getIndex(), working_tentacle.getIndex()));
			//If we hit the goal, make the fitness infinate, since we can't break from an OpenMP loop
			if(hit_goal)
			{
				tent_fitness = std::numeric_limits<double>::max();
			}
			best_tentacle_candidates.push(tent_fitness, tent_details);
		}
	}

	speedset_idx = best_tentacle_candidates.top()->first;
	tentacle_idx = best_tentacle_candidates.top()->second;

	ROS_INFO("Tentacles Searched: %d, I Selected Speed Set %d, Tentacle %d", best_tentacle_candidates.size(), speedset_idx, tentacle_idx);

	return true;
}

void LocalPlanner::planningCB(const ros::TimerEvent& event)
{
	if(should_plan_)
	{
		ROS_INFO("I'm Planning!");
		//Grab the next occupancy grid to process
		if(!this->occupancy_buffer_.empty())
		{
			OccupancyGrid	working_grid= this->occupancy_buffer_.front();
			int speedset_idx = 0;
			int tentacle_idx = 0;

			//select the best tentacle
			this->selectTentacle(0, working_grid, speedset_idx, tentacle_idx);
			//Update the current radius and velocity
			this->set_rad_ = this->tentacles_->getSpeedSet(speedset_idx).getTentacle(tentacle_idx).getRad();
			this->set_vel_ = this->tentacles_->getSpeedSet(speedset_idx).getTentacle(tentacle_idx).getVel();
			visualizeTentacle(speedset_idx, tentacle_idx);
			this->occupancy_buffer_.pop_front();
		}
	}
}

void LocalPlanner::velUpdateCB(const ros::TimerEvent& event)
{
	ROS_INFO("I'm Updating Velocity!");
	this->sendVelCom(this->set_vel_, this->set_rad_);
}

void LocalPlanner::stopCB(const aero_srr_msgs::SoftwareStopConstPtr& message){
	//Need to flip as message is true when should stop
	this->should_plan_ = !message->stop;
	ROS_WARN("Oryx Local Path Planner Received A Software Stop Message [%s]: %s",(message->stop)?"Stop":"Go", message->message.c_str());
	//If we were told to stop, clear the buffer so we don't process stale data at wakeup
	if(!this->should_plan_)this->occupancy_buffer_.clear();
}


void LocalPlanner::pcCB(const aero_path_planning::OccupancyGridMsgConstPtr& message){
	//To prevent processing of stale data, ignore anything received while we shouldn't be planning
	if(this->should_plan_)
	{
		OccupancyGrid recievedGrid(*message);
		//ROS_INFO("Grid Processed...");
		//ROS_INFO("Callback got the grid:\n%s", recievedGrid.toString(0,0)->c_str());
		this->occupancy_buffer_.push_back(recievedGrid);
	}
}


void LocalPlanner::sendVelCom(double velocity, double radius)
{
	switch (this->platform_)
	{
	case 0:
		ROS_ERROR("Platform Oryx is no longer supported");
		break;
	case 1:
		twist(velocity, velocity/(radius/10.0));
		break;
	default:
		break;
	}
}


void LocalPlanner::twist(double x_dot, double omega)
{
	geometry_msgs::Twist message;
	if(std::abs(x_dot)<0.01)
	{
		x_dot = 0;
	}
	else if(std::abs(x_dot)<0.15)
	{
		x_dot = x_dot/std::abs(x_dot)*0.15;
	}

	if(std::abs(omega)<0.01)
	{
		omega = 0;
	}
	else if(std::abs(omega)<0.1)
	{
		omega = omega/std::abs(omega)*0.1;
	}
	message.linear.x  = x_dot;
	message.angular.z = omega;
	this->vel_pub_.publish(message);
}

void LocalPlanner::visualizeTentacle(int speed_set, int tentacle)
{
	sensor_msgs::PointCloud2 message;
	OccupancyGridCloud cloud(this->tentacles_->getSpeedSet(speed_set).getTentacle(tentacle).getPoints());
	PointConverter converter(this->res_);
	for(int i=0; i<cloud.size(); i++)
	{
		Point& point = cloud.at(i);
		converter.convertToEng(point, point);
	}
	pcl::toROSMsg(cloud , message);
	message.header.frame_id = "/laser";
	message.header.stamp    = ros::Time::now();
	this->tent_pub_.publish(message);
}

