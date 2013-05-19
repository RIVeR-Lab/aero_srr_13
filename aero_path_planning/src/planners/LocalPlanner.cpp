/**
 * @file LocalPlanner.cpp
 *
 * @date Mar 9, 2013
 * @author Adam Panzica
 */

//********************** SYSTEM DEPENDANCIES **********************//
#include <geometry_msgs/PointStamped.h>
//********************** LOCAL  DEPENDANCIES **********************//
#include <aero_path_planning/planners/LocalPlanner.h>
#include <aero_path_planning/utilities/FitnessQueue.h>
#include "OryxPathPlannerConfig.h"
#include <aero_path_planning/occupancy_grid/MultiTraitOccupancyGridHelpers.h>

using namespace aero_path_planning;



LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& p_nh) throw(std::runtime_error):
																																				nh_(nh),
																																				p_nh_(p_nh),
																																				limiter_(NULL),
																																				occupancy_buffer_(2)
{
	ROS_INFO("Starting Up Aero Local Planner Version %d.%d.%d", oryx_path_planner_VERSION_MAJOR, oryx_path_planner_VERSION_MINOR, oryx_path_planner_VERSION_BUILD);

	this->dr_server_.setCallback(boost::bind(&LocalPlanner::drCB, this, _1, _2));

	this->current_rad_ = 0;
	this->current_vel_ = 0;

	this->loadParam();
	this->regTopic();
	this->regTimers();
	this->setTentacleMode();

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
	this->pc_topic_       = OCCUPANCY_TOPIC;
	this->state_topic_    = STATE_TOPIC;
	this->man_topic_      = MAN_TOPIC;
	this->lidar_topic_    = LIDAR_LOCAL_TOPIC;


	//*****************Configuration Parameters*******************//
	//The platform that the local planner is running on
	std::string p_platform(PLATFORM);
	this->platform_ = 0;
	std::string platform_message("Oryx");

	//*****************Configuration Parameters*******************//
	//Minimum update rate expected of occupancy grid
	std::string p_up_rate(L_OCC_UPDTRT);
	double update_rate = 0.2;
	std::stringstream up_rate_msg;
	up_rate_msg<<update_rate<<"s";

	//x dimension of occupancy grid
	std::string p_x_dim(L_OCC_XDIM);
	this->x_dim_ = 200;
	std::stringstream x_dim_msg;
	x_dim_msg<<this->x_dim_<<"0.05m";

	//y dimension of occupancy grid
	std::string p_y_dim(L_OCC_YDIM);
	this->y_dim_ = 200;
	std::stringstream y_dim_msg;
	y_dim_msg<<this->y_dim_<<"0.05m";

	//z dimension of occupancy grid
	std::string p_z_dim(L_OCC_ZDIM);
	this->z_dim_ = 0;
	std::stringstream z_dim_msg;
	z_dim_msg<<this->z_dim_<<"0.05m";

	//resolution occupancy grid
	std::string p_res(L_OCC_RES);
	this->res_ = .05;
	std::stringstream p_res_msg;
	p_res_msg<<this->res_<<"m";

	//x coord of the origin of the occupancy grids
	std::string p_x_ori(L_OCC_XORG);
	double x_ori = 0;
	std::stringstream p_x_ori_msg;
	p_x_ori_msg<<x_ori<<"m";

	//z coord of the origin of the occupancy grids
	std::string p_z_ori(L_OCC_ZORG);
	double z_ori = 0;
	std::stringstream p_z_ori_msg;
	p_z_ori_msg<<z_ori<<"m";

	//y coord of the origin of the occupancy grids
	std::string p_y_ori(L_OCC_YORG);
	double y_ori = this->y_dim_/2;
	std::stringstream p_y_ori_msg;
	p_y_ori_msg<<y_ori<<"m";


	//Minimum length of tentacles
	std::string p_min_tent(T_MINLNTH);
	double min_tent = 10;
	std::stringstream p_min_tent_msg;
	p_min_tent_msg<<min_tent<<"m";

	//number of tentacles per speed set
	std::string p_num_tent(T_NUMBER);
	int num_tent = 81;
	std::stringstream p_num_tent_msg;
	p_num_tent_msg<<num_tent<<" Tentacles";

	//Exponential Factor to use for generating seed radii
	std::string p_exp_fact(T_EXPFACT);
	double exp_fact = 1.15;
	std::stringstream p_exp_fact_msg;
	p_exp_fact_msg<<exp_fact;

	//number of speed sets
	std::string p_num_speed_set(S_NUMBER);
	int num_speed_set = 15;
	std::stringstream p_numSpeedSet_msg;
	p_numSpeedSet_msg<<num_speed_set<<" Speed Sets";

	//Max Speed
	std::string p_max_speed(S_MAX_SPEED);
	double max_speed = 1.5;
	std::stringstream p_max_speed_msg;
	p_max_speed_msg<<max_speed<<"m/s";

	//Min Speed
	std::string p_min_speed(S_MIN_SPEED);
	double min_speed = .75;
	std::stringstream p_min_speed_msg;
	p_min_speed_msg<<min_speed<<"m/s";

	//Goal Weight
	std::string p_goal_weight(GOAL_WEIGHT);
	double goal_weight = 1.0;
	std::stringstream p_goal_weight_msg;
	p_goal_weight_msg<<goal_weight;

	//Traversed Weight
	std::string p_trav_weight(TRAV_WEIGHT);
	double trav_weight = 0.1;
	std::stringstream p_trav_weight_msg;
	p_trav_weight_msg<<trav_weight;

	//Difficulty Weight
	std::string p_diff_weight(DIFF_WEIGHT);
	double diff_weight = 0.1;
	std::stringstream p_diff_weight_msg;
	p_diff_weight_msg<<diff_weight;

	//Unkown Terrain Weight
	std::string p_unkn_weight(UNKN_WEIGHT);
	double unkn_weight = 0.1;
	std::stringstream p_unkn_weight_msg;
	p_unkn_weight_msg<<unkn_weight;

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
	if(!p_nh_.getParam(this->v_action_topic_,  this->v_action_topic_))	PARAM_WARN(this->v_action_topic_, this->v_action_topic_);
	if(!p_nh_.getParam(this->pc_topic_,	       this->pc_topic_))	    PARAM_WARN(this->pc_topic_,	      this->pc_topic_);
	if(!p_nh_.getParam(this->state_topic_,     this->state_topic_))	    PARAM_WARN(this->state_topic_,    this->state_topic_);
	if(!p_nh_.getParam(this->man_topic_,	   this->man_topic_))	    PARAM_WARN(this->man_topic_,	  this->man_topic_);
	if(!p_nh_.getParam(this->lidar_topic_,	   this->lidar_topic_))	    PARAM_WARN(this->lidar_topic_,	  this->lidar_topic_);


	if(!p_nh_.getParam(p_goal_weight,	this->goal_weight_))    PARAM_WARN(p_goal_weight, p_goal_weight_msg.str());
	if(!p_nh_.getParam(p_trav_weight,	this->trav_weight_))    PARAM_WARN(p_trav_weight, p_trav_weight_msg.str());
	if(!p_nh_.getParam(p_diff_weight,	this->diff_weight_))    PARAM_WARN(p_diff_weight, p_diff_weight_msg.str());
	if(!p_nh_.getParam(p_unkn_weight,	this->unkn_weight_))    PARAM_WARN(p_unkn_weight, p_unkn_weight_msg.str());
	//Get Public Parameters
	if(!nh_.getParam(p_up_rate,	update_rate))       PARAM_WARN(p_up_rate,	up_rate_msg.str());
	if(!nh_.getParam(p_x_dim,	this->x_dim_))		 PARAM_WARN(p_x_dim,	x_dim_msg.str());
	if(!nh_.getParam(p_y_dim,	this->y_dim_))		 PARAM_WARN(p_y_dim,	y_dim_msg.str());
	if(!nh_.getParam(p_z_dim,	this->z_dim_))		 PARAM_WARN(p_z_dim,	z_dim_msg.str());
	if(!nh_.getParam(p_x_ori,	x_ori))             PARAM_WARN(p_x_ori,	p_x_ori_msg.str());
	if(!nh_.getParam(p_y_ori,	y_ori))             PARAM_WARN(p_y_ori,	p_y_ori_msg.str());
	if(!nh_.getParam(p_z_ori,	z_ori))			 PARAM_WARN(p_z_ori,	p_z_ori_msg.str());
	if(!nh_.getParam(p_res,		this->res_))		 PARAM_WARN(p_res,		p_res_msg.str());
	if(!nh_.getParam(p_min_tent,	min_tent))		 PARAM_WARN(p_min_tent,	p_min_tent_msg.str());
	if(!nh_.getParam(p_num_tent,	num_tent))		 PARAM_WARN(p_num_tent,	p_num_tent_msg.str());
	if(!nh_.getParam(p_exp_fact,	exp_fact))		 PARAM_WARN(p_exp_fact,	p_exp_fact_msg.str());
	if(!nh_.getParam(p_num_speed_set, num_speed_set))PARAM_WARN(p_num_speed_set,	p_numSpeedSet_msg.str());
	if(!nh_.getParam(p_max_speed,	max_speed))    PARAM_WARN(p_max_speed,	     p_max_speed_msg.str());
	if(!nh_.getParam(p_min_speed,	min_speed))    PARAM_WARN(p_min_speed,	     p_min_speed_msg.str());
	this->origin_.x = x_ori;
	this->origin_.y = y_ori;
	this->origin_.z = z_ori;
	this->tentacles_ = TentacleGeneratorPtr(new TentacleGenerator(min_tent, min_speed,max_speed,num_speed_set, num_tent, exp_fact, this->res_, this->x_dim_, this->y_dim_));

	std::string p_rate_limit("rate_limit");
	this->rate_limit_ = 5;
	this->limiter_   = new TentacleRateLimiter(num_tent-1, this->rate_limit_);
}

void LocalPlanner::regTopic()
{
	this->pc_sub_    = this->nh_.subscribe(this->pc_topic_,    1, &LocalPlanner::pcCB,    this);
	this->state_sub_ = this->nh_.subscribe(this->state_topic_, 1, &LocalPlanner::stateCB, this);
	this->joy_sub_   = this->nh_.subscribe(this->man_topic_,   1, &LocalPlanner::manTwistCB,   this);
	this->lidar_sub_ = this->nh_.subscribe(this->lidar_topic_, 1, &LocalPlanner::lidarCB, this);
	this->vel_pub_   = this->nh_.advertise<geometry_msgs::Twist>(this->v_action_topic_, 1);
	this->tent_pub_  = this->nh_.advertise<sensor_msgs::PointCloud2>("/aero/tencale_visualization", 1);
	this->occ_viz_pub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>("/aero/local/occupancy_viz",1);
	this->goal_sub_  = this->nh_.subscribe("/aero/global/goal", 1, &LocalPlanner::goalCB, this);

	std::string software_stop_topic("aero/software_stop");

	if(!this->nh_.getParam(software_stop_topic, software_stop_topic))ROS_WARN("%s not set, using default value %s", software_stop_topic.c_str(), software_stop_topic.c_str());
	this->stop_sub_ = nh_.subscribe(software_stop_topic, 1, &LocalPlanner::stopCB, this);
}

void LocalPlanner::regTimers()
{
	this->plan_period_ = ros::Duration(1.0/20.0);
	this->vel_period_  = this->plan_period_;
	this->vel_timer_ = nh_.createTimer(this->vel_period_, &LocalPlanner::velUpdateCB, this);
	this->plan_timer_= nh_.createTimer(this->plan_period_, &LocalPlanner::planningCB, this);
}

LocalPlanner::~LocalPlanner()
{
	delete this->limiter_;
};


void LocalPlanner::goalCB(const geometry_msgs::PoseStampedConstPtr& message)
{
	this->global_goal_ = message;
}

bool LocalPlanner::selectTentacle(const double& current_vel, const og::MultiTraitOccupancyGrid& search_grid, int& speedset_idx, int& tentacle_idx)
{
	typedef std::pair<int, int> TentacleData_t;
	typedef boost::shared_ptr<TentacleData_t > TentacleDataPtr_t;
	FitnessQueue<double,  TentacleDataPtr_t> best_tentacle_candidates;
	std::vector<SpeedSet> sets;
	app::PointConverter converter(this->res_);

	SpeedSet current_set = this->tentacles_->getSpeedSet(this->current_vel_);
	sets.push_back(current_set);

	if(current_set.getIndex()!=0)
	{
		SpeedSet slow_set    = this->tentacles_->getSpeedSet(current_set.getIndex()-1);
		sets.push_back(slow_set);
	}
	if(current_set.getIndex()<this->tentacles_->getNumSpeedSets())
	{
		SpeedSet fast_set    = this->tentacles_->getSpeedSet(current_set.getIndex()+1);
		sets.push_back(fast_set);
	}
	bool   has_goal      = true;
	bool   hit_goal      = false;
	for(int s=0; s<(int)sets.size(); s++)
	{
		SpeedSet cur_set(sets.at(s));
		//#pragma omp parallel for
		for(int i=0; i<(int)current_set.getNumTentacle(); i++)
		{
			//If we already hit the goal, short circuit since we can't break from OpenMP loops
			if(!hit_goal)
			{
				double length_modifier = 0;
				bool traversing = true;
				Tentacle working_tentacle = cur_set.getTentacle(i);
				Tentacle::TentacleTraverser traverser(working_tentacle);
				length_modifier = 0;

				//As long as the tentacle has points still, and we're still traversing, continue traversing
				app::Point      temp_point;
				gm::PoseStamped point_pose;
				while(traverser.hasNext()&&traversing)
				{
					converter.convertToEng(traverser.next(), temp_point);
					app::pointToPose(temp_point, point_pose.pose);
					try
					{
						switch(search_grid.getPointTrait(point_pose).getEnum())
						{
						case ogu::CellTrait::OBSTACLE:
							//ROS_INFO("Hit Obstacle On Tentacle %d at length %f", i, traverser.lengthTraversed());
							//PRINT_POINT("Hit Point", point_pose);
							traversing = false;
							break;
						case ogu::CellTrait::GOAL:
							//ROS_INFO("Hit the Goal on Tentacle %d at length %f", i, traverser.lengthTraversed());
							traversing = false;
							hit_goal   = true;
							break;
						case ogu::CellTrait::FREE_HIGH_COST:
							length_modifier -= traverser.deltaLength()*this->diff_weight_;
							break;
						case ogu::CellTrait::TRAVERSED:
							length_modifier -= traverser.deltaLength()*this->trav_weight_;
							break;
						case ogu::CellTrait::UNKOWN:
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
						//Will throw false if there was no goal point_pose
						gm::Pose goal_pose;
						search_grid.getGoal(goal_pose);
						aero_path_planning::Point goal_point;
						app::poseToPoint(goal_pose, goal_point);
						converter.convertToGrid(goal_point, goal_point);
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

				double raw_length   = traverser.lengthTraversed();
				double tent_fitness = 0;
				//Cuttoff for valid tentacle (to stop a goal inside a wall making the robot drive into it)
				if(raw_length>.25/this->res_)
				{
					tent_fitness = raw_length+length_modifier;
				}
				//ROS_INFO("Searched Tentacle %d in set %d with fitness %f",current_set.getIndex(), working_tentacle.getIndex(), tent_fitness);
				TentacleDataPtr_t tent_details(new TentacleData_t(cur_set.getIndex(), working_tentacle.getIndex()));
				//If we hit the goal, make the fitness infinate, since we can't break from an OpenMP loop
				if(hit_goal)
				{
					tent_fitness = std::numeric_limits<double>::max();
				}
				best_tentacle_candidates.push(tent_fitness, tent_details);
			}
		}
	}

	speedset_idx = best_tentacle_candidates.top()->first;
	tentacle_idx = best_tentacle_candidates.top()->second;

	//ROS_INFO("Tentacles Searched: %d, I Selected Speed Set %d, Tentacle %d", best_tentacle_candidates.size(), speedset_idx, tentacle_idx);

	return true;
}

void LocalPlanner::planningCB(const ros::TimerEvent& event)
{
	if(event.profile.last_duration.toSec()>this->plan_period_.toSec()*1.1)
	{
		ROS_WARN_STREAM_THROTTLE(1, "Local Planner Callback is taking longer than its timer period to process. Alloted Period="<<this->plan_period_<<"s, Actual Period="<<event.profile.last_duration);
	}


	if(should_plan_)
	{
		//Grab the next occupancy grid to process if we've recieved new data from global planner
		if(!this->occupancy_buffer_.empty())
		{
			this->working_grid_= this->occupancy_buffer_.front();
			this->occupancy_buffer_.pop_front();
		}
		if(this->working_grid_!=occupancy_grid::MultiTraitOccupancyGridPtr())
		{
			//Build a working grid to apply the LIDAR patch to
			og::MultiTraitOccupancyGrid working_grid(*this->working_grid_);
			//If we actually have a working grid, plan on it
			//ROS_INFO_STREAM("I Have  Working Grid to Local Plan On!");
			//If we have a LIDAR patch, apply it
			if(this->lidar_patch_!= PointCloudPtr())
			{
				//ROS_INFO_STREAM("I'm apllying the LIDAR pach...");
				//ros::Time lidarStart(ros::Time::now());
				std::string error_message;
				bool success = app::addPointCloudPatch(*this->lidar_patch_, ogu::CellTrait::OBSTACLE, 1000, working_grid, this->transformer_, error_message);
				//ROS_INFO_STREAM("Lidar Copying Took "<<ros::Time::now()-lidarStart<<" seconds");
				//ROS_INFO_STREAM("Patch Applied"<<success<<"!");
				if(!success)
				{
					ROS_INFO_STREAM(error_message);
				}
			}

			//Apply a goal if we have one:
			this->applyGoal(working_grid);

			//Visualize the grid
			this->visualizeOcc(working_grid);

			//Check to see if we're at the goal
			double dist;
			geometry_msgs::Pose local_goal;
			if(working_grid.getGoal(local_goal))
			{
				tf::Vector3 goal_position;
				tf::pointMsgToTF(local_goal.position, goal_position);
				dist = goal_position.length();
				ROS_INFO_STREAM_THROTTLE(1, "Distance to Local Goal:"<<dist<<", goal <"<<goal_position.x()<<","<<goal_position.y()<<">");
			}
			else
			{
				//Means we have no goal, we can't be at it
				dist = std::numeric_limits<double>::infinity();
			}
			//Check to see if we got within a threshold distance of the local goal
			if(dist>=0.25)
			{
				int speedset_idx = 0;
				int tentacle_idx = 0;
				//select the best tentacle
				ros::Time selectStart(ros::Time::now());
				this->selectTentacle(0, working_grid, speedset_idx, tentacle_idx);
				//ROS_INFO_STREAM("Tentacle Selection Took "<<ros::Time::now()-selectStart<<" seconds");
				//Limit rate of tentacle change
				tentacle_idx   = this->limiter_->nextTentacle(tentacle_idx);
				//Update the current radius and velocity
				this->set_rad_ = -this->tentacles_->getSpeedSet(speedset_idx).getTentacle(tentacle_idx).getRad();
				this->set_vel_ = this->tentacles_->getSpeedSet(speedset_idx).getTentacle(tentacle_idx).getVel();
				visualizeTentacle(speedset_idx, tentacle_idx);
			}
			else
			{
				this->set_rad_ = 0;
				this->set_vel_ = 0;
			}
		}
		else
		{
			ROS_WARN_STREAM("I Have  No Working Grid To Plan On!");
			this->set_vel_ = 0;
			this->set_rad_ = 0;
		}

	}
	else
	{
		this->set_vel_ = 0;
		this->set_rad_ = 0;
	}
}

void LocalPlanner::applyGoal(og::MultiTraitOccupancyGrid& grid) const
{
	geometry_msgs::PoseStamped local_goal;
	if(this->global_goal_!=geometry_msgs::PoseStampedConstPtr())
	{
		try
		{
			PointConverter converter(this->res_);
			this->transformer_.waitForTransform(grid.getFrameID(), this->global_goal_->header.frame_id, grid.getCreationTime(), ros::Duration(1.0/20.0));
			this->transformer_.transformPose(grid.getFrameID(), grid.getCreationTime(), *this->global_goal_, this->global_goal_->header.frame_id, local_goal);

			grid.setGoal(local_goal.pose);

		}
		catch(std::exception& e)
		{
			ROS_ERROR_STREAM_THROTTLE(1, e.what());
		}
	}
}

void LocalPlanner::velUpdateCB(const ros::TimerEvent& event)
{
	if(event.profile.last_duration.toSec()>this->vel_period_.toSec()*1.1)
	{
		ROS_WARN_STREAM_THROTTLE(1, "Local Planner Velocity Update is taking longer than its timer period to process. Alloted Period="<<this->vel_period_<<"s, Actual Period="<<event.profile.last_duration);
	}
	this->sendVelCom(this->set_vel_, this->set_rad_);
}

void LocalPlanner::stopCB(const aero_srr_msgs::SoftwareStopConstPtr& message){
	//Need to flip as message is true when should stop
	if(message->stop)
	{
		this->setSafeMode(true);
	}
	else
	{
		this->setSafeMode(false);
	}
	ROS_WARN("Oryx Local Path Planner Received A Software Stop Message [%s]: %s",(message->stop)?"Stop":"Go", message->message.c_str());
	//If we were told to stop, clear the buffer so we don't process stale data at wakeup
	if(!this->should_plan_)this->occupancy_buffer_.clear();
}


void LocalPlanner::pcCB(const og::MultiTraitOccupancyGridMessageConstPtr& message){
	//To prevent processing of stale data, ignore anything received while we shouldn't be planning
	if(this->should_plan_)
	{
		//ROS_INFO("Grid Processed...");
		//ROS_INFO("Callback got the grid:\n%s", recievedGrid.toString(0,0)->c_str());
		this->occupancy_buffer_.push_back(og::MultiTraitOccupancyGridPtr(new og::MultiTraitOccupancyGrid(*message)));
	}
}

void LocalPlanner::manTwistCB(const geometry_msgs::TwistConstPtr& message)
{
	//Check to make sure we're in manual mode and weapons free
	if(!this->tentacle_mode_&&this->should_plan_)
	{
		this->twist(message->linear.x, message->angular.z);
	}
	else
	{
		if(!this->should_plan_)
		{
			ROS_WARN_STREAM_THROTTLE(5, "Cannot Move, Local Planner is in Safe Mode!");
		}
	}
}

void LocalPlanner::stateCB(const aero_srr_msgs::AeroStateConstPtr& message)
{
	typedef aero_srr_msgs::AeroState State_t;
	switch(message->state)
	{
	case State_t::STARTUP:
	case State_t::ERROR:
	case State_t::SAFESTOP:
	case State_t::SHUTDOWN:
		this->setSafeMode(true);
		break;
	case State_t::SEARCH:
	case State_t::NAVOBJ:
	case State_t::HOME:
		this->setTentacleMode();
		break;
	case State_t::MANUAL:
	case State_t::COLLECT:
		this->setManualMode();
		break;
	default:
		ROS_ERROR_STREAM("Received Unknown Robot State: "<<message->state);
		break;
	}
}

void LocalPlanner::lidarCB(const sensor_msgs::PointCloud2ConstPtr& message)
{
	PointCloudPtr lidar_patch(new PointCloud());
	pcl::PointCloud<pcl::PointXYZ> raw_cloud;
	pcl::fromROSMsg(*message, raw_cloud);
	PointConverter converter(this->res_);

	//#pragma omp parallel for
	for(int i=0; i<(int)raw_cloud.size(); i++)
	{
		Point point;
		point.x = raw_cloud.at(i).x;
		point.y = raw_cloud.at(i).y;
		point.z = 0;
		point.rgba = OBSTACLE;
		converter.convertToGrid(point, point);
		if(this->boundsCheck(point))
		{
			lidar_patch->push_back(point);
		}
	}

	this->lidar_patch_ = lidar_patch;
}


void LocalPlanner::sendVelCom(double velocity, double radius)
{
	switch (this->platform_)
	{
	case 0:
		ROS_ERROR_THROTTLE(1,"Sending Velocity To Platform Oryx is no longer supported");
		break;
	case 1:
		twist(velocity, velocity/radius);
		break;
	default:
		break;
	}
}

void LocalPlanner::drCB(const LocalPlannerConfig& config, uint32_t levels)
{
	this->goal_weight_ = config.goal_weight;
	this->unkn_weight_ = config.unkown_weight;
	this->diff_weight_ = config.difficult_weight;
	this->trav_weight_ = config.traversed_weight;
	if(this->limiter_!=NULL)
	{
		if(config.rate_limit>0)
		{
			this->limiter_->disableLimit(false);
			this->rate_limit_  = config.rate_limit;
		}
		else
		{
			this->limiter_->disableLimit(true);
		}
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
	message.angular.z = omega*2.0;
	this->vel_pub_.publish(message);
}

void LocalPlanner::visualizeTentacle(int speed_set, int tentacle)
{
	sensor_msgs::PointCloud2 message;
	OccupancyGridCloud cloud(this->tentacles_->getSpeedSet(speed_set).getTentacle(tentacle).getPoints());
	PointConverter converter(this->res_);
#pragma omp parallel for
	for(int i=0; i<(int)cloud.size(); i++)
	{
		Point& point = cloud.at(i);
		converter.convertToEng(point, point);
	}
	pcl::toROSMsg(cloud , message);
	message.header.frame_id = "/base_footprint";
	message.header.stamp    = ros::Time::now();
	this->tent_pub_.publish(message);
}

void LocalPlanner::visualizeOcc(const og::MultiTraitOccupancyGrid& grid)
{

	nav_msgs::OccupancyGridPtr message(new nav_msgs::OccupancyGrid);
	grid.generateOccupancyGridforTrait(*message, ogu::CellTrait::OBSTACLE);
	this->occ_viz_pub_.publish(message);
}


void LocalPlanner::setManualMode()
{
	ROS_INFO_STREAM("Local Planner Set to Manual Control!");
	this->setSafeMode(false);
	this->tentacle_mode_= false;
	this->plan_timer_.stop();
	this->vel_timer_.stop();
}

void LocalPlanner::setTentacleMode()
{
	ROS_INFO_STREAM("Local Planner Set to Tentacles Control!");
	this->setSafeMode(false);
	this->tentacle_mode_ = true;
	this->plan_timer_.start();
	this->vel_timer_ .start();
}

void LocalPlanner::setSafeMode(bool safe)
{
	std::string mode(safe?("Safe"):("Free"));
	ROS_INFO_STREAM("Local Planner Set to "<<mode<<" Mode!");
	this->should_plan_ = !safe;
}

bool LocalPlanner::boundsCheck(const Point& point) const
{
	Point corrected_point(point);
	corrected_point.getVector4fMap()+=this->origin_.getVector4fMap();
	bool inx = (corrected_point.x <= this->x_dim_)&&(corrected_point.x>=0);
	bool iny = (corrected_point.y <= this->y_dim_)&&(corrected_point.y>=0);
	bool inz = (corrected_point.z <= this->z_dim_)&&(corrected_point.z>=0);
	return inx&&iny&&inz;
}
