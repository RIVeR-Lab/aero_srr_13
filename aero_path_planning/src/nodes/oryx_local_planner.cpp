/**
 * @file	oryx_base_planner.cpp
 * @date	Oct 23, 2012
 * @author	Adam Panzica
 * @brief	Planning node for doing local planning and movement operations
 */

//*********************** SYSTEM DEPENDENCIES ************************************//
#include<queue>
#include<actionlib/client/simple_action_client.h>
#include<aero_drive_controller/VelocityCommandAction.h>
#include<boost/lexical_cast.hpp>
#include<boost/circular_buffer.hpp>
#include<oryx_msgs/SoftwareStop.h>
#include<geometry_msgs/Twist.h>
//*********************** LOCAL DEPENDENCIES ************************************//
#include <aero_path_planning/OryxPathPlanning.h>
#include "OryxPathPlannerConfig.h"
#include <aero_path_planning/OccupancyGridMsg.h>

//*********************** MACROS ************************************//
///Number of seconds to wait for connections to other ROS nodes before determining a system failure
#define CONNECTION_TIMEOUT	5.0

using namespace aero_path_planning;

//*********************** HELPER CLASS DEFINITIONS ******************************//

/**
 * @author	Adam Panzica
 * @brief	Class which does the actual work of local path planning and sending commands to base
 */
class LocalPlanner
{
public:
	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new local planner
	 * @param platform				integer denoting what platform the local planner is running on
	 * @param goalWeight			weighting factor to bias tentacle selection towards the goal point
	 * @param travWeight			weighting factor to bias tentacle selection away from previously traversed points
	 * @param diffWeight			weighting factor to bias tentacle selection away from difficult terrain
	 * @param unknWeight			weighting factor to bias tentacle selection towards from unkown terrain
	 * @param xDim					x dimension of the occupancy grids to use, in real units
	 * @param yDim					y dimension of the occupancy grids to use, in real units
	 * @param zDim					z dimension of the occupancy grids to use, in real units
	 * @param res					resolution  of the occupancy grids to use, in real units per grid unit
	 * @param origin				Origin to use for the occupancy grid
	 * @param v_action_topic		Actionlib topic to communicate with the base platform
	 * @param oc_point_cloud_topic	Topic name of the ROS topic to receive new occupancy grid data over
	 * @param tentacles				Pointer to the tentacle generator which contains the tentacles to use for planning
	 * @param v_client				Pointer to the velocity client to send commands to the base platform
	 * @throw std::runtime_error	If the planner is unable to connect to the base platform
	 */
	LocalPlanner(int platform, double goalWeight, double travWeight, double diffWeight, double unknWeight, double xDim, double yDim, double zDim, double res, aero_path_planning::Point& origin, std::string& v_action_topic, std::string& oc_point_cloud_topic, TentacleGeneratorPtr tentacles) throw(std::runtime_error):
		v_action_topic_(v_action_topic),
		pc_topic_(oc_point_cloud_topic),
		origin_(origin),
		tentacles_(tentacles),
		occupancy_buffer_(2),
		v_client_(v_action_topic, true)
{
		this->platform_    = platform;
		this->goal_weight_ = goalWeight;
		this->trav_weight_ = travWeight;
		this->diff_weight_ = diffWeight;
		this->unkn_weight_ = unknWeight;
		this->x_dim_ = xDim;
		this->y_dim_ = yDim;
		this->z_dim_ = zDim;
		this->res_  = res;
		this->pc_sub_ = nh_.subscribe(this->pc_topic_, 1, &LocalPlanner::pcCB, this);
		this->tent_pub_= nh_.advertise<sensor_msgs::PointCloud2>("/aero/tencale_visualization", 2);
		this->should_plan_ = true;
		this->current_rad_ = 0;
		this->current_vel_ = 0;

		std::string software_stop_topic("oryx/software_stop");

		if(!this->nh_.getParam(software_stop_topic, software_stop_topic))ROS_WARN("%s not set, using default value %s", software_stop_topic.c_str(), software_stop_topic.c_str());
		this->stop_sub_ = nh_.subscribe(software_stop_topic, 1, &LocalPlanner::stopCB, this);

		switch(this->platform_)
		{
		case 0:
			ROS_INFO("Waiting For Connection To Velocity Control Server On <%s>...", this->v_action_topic_.c_str());
			if(v_client_.waitForServer(ros::Duration(CONNECTION_TIMEOUT)))
			{
				ROS_INFO("Connection Established");
			}
			else
			{
				ROS_ERROR("Could Not Establish Connection To Velocity Control Server!");
				std::string errMsg("Unable to connect to velocity command server over ");
				throw std::runtime_error(errMsg+this->v_action_topic_);
			}
			break;
		case 1:
			ROS_INFO_STREAM("Publishing Twists to <"<<this->v_action_topic_<<">");
			this->vel_pub_ = this->nh_.advertise<geometry_msgs::Twist>(this->v_action_topic_, 2);
			break;
		default:
			ROS_FATAL("Unkown Platform, Could Not Setup Local Planner");
			break;

		}
		ROS_INFO("Oryx Local Planner Running");
}
	/**
	 * default destructor
	 */
	virtual ~LocalPlanner(){};

	/**
	 * @author	Adam Panzica
	 * @brief	Does the actual work of implementing the Driving with Tentacles algorithm
	 *
	 */
	void doPlanning()
	{
		Tentacle zero_tentacle(0,0,0,0,0,0,0,0,0);
		double current_radius = std::numeric_limits<double>::infinity();
		double current_velocity = 0;
		while(ros::ok())
		{
			if(should_plan_)
			{
				//Grab the next occupancy grid to process
				if(!this->occupancy_buffer_.empty())
				{
					OccupancyGrid	working_grid= this->occupancy_buffer_.front();
					SpeedSet	  	speed_set   = this->tentacles_->getSpeedSet(2);

					//ROS_INFO("I'm Processing The Following Occupancy Grid:\n%s", working_grid.toString(0,0).get()->c_str());
					int num_points = 0;
					for(OccupancyGrid::iterator test_itr = working_grid.begin(); test_itr<working_grid.end(); test_itr++)
					{
						//ROS_INFO_COND(test_itr->rgba != aero_path_planning::FREE_LOW_COST, "Found an obsticale point! <%f, %f>", test_itr->x, test_itr->y);
						if(test_itr->rgba != aero_path_planning::FREE_LOW_COST)num_points++;
					}
					ROS_INFO("I Found %d number of interesting points", num_points);



					ROS_INFO("I'm Going To Use A Speed Set With The Parameters <NumTent:%d, Vel:%f, SR:%f>", speed_set.getNumTentacle(), speed_set.getVelocity(), speed_set.getSeedRad());
					OccupancyGrid rendering(working_grid);
					unsigned int longest_index = 0;
					double longest_length = 0;
					double length_modifier = 0;
					bool   has_goal      = true;
					ROS_INFO("I'm looking for the longest tentacle...");

					for(unsigned int i=0; i<speed_set.getNumTentacle(); i++)
					{
						bool traversing = true;
						bool hit_goal   = false;
						Tentacle working_tentacle = speed_set.getTentacle(i);
						Tentacle::TentacleTraverser traverser(working_tentacle);
						length_modifier = 0;

						//As long as the tentacle has points still, and we're still traversing, continue traversing
						while(traverser.hasNext()&&traversing)
						{
							const aero_path_planning::Point& point = traverser.next();
							try
							{
								switch(working_grid.getPointTrait(point))
								{
								case aero_path_planning::OBSTACLE:
									//ROS_INFO("Hit Obstacle On Tentacle %d at length %f", i, traverser.lengthTraversed());
									PRINT_POINT("Hit Point", point);
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
							ROS_INFO_STREAM("There is a goal...");
							try
							{
								//Will throw false if there was no goal point
								const aero_path_planning::Point goal_point = working_grid.getGoalPoint();
								const aero_path_planning::Point end_point = traverser.next();
								double dist_to_goal = pcl::distances::l2(end_point.getVector4fMap(), goal_point.getVector4fMap());
								ROS_INFO_STREAM("Distance To Goal: "<<dist_to_goal);
								length_modifier-= dist_to_goal*this->goal_weight_;
							}
							catch(bool& e)
							{
								ROS_INFO("There is not a goal");
								//There was no goal, set the flag
								has_goal = e;
							}
						}

						//Check to see if the current tentacle is the best one
						if(traverser.lengthTraversed()+length_modifier >= (longest_length))
						{
							longest_index = i;
							longest_length = traverser.lengthTraversed();
							//if we hit the goal, break out of the tentacle search
							if(hit_goal) break;
						}
					}
					//Update the current radius and velocity
					current_radius   = speed_set.getTentacle(longest_index).getRad();
					current_velocity = speed_set.getTentacle(longest_index).getVel();
					//Print out the selected tentacle on the grid
					ROS_INFO("I Selected Tentacle %d, length %f", longest_index, longest_length);
					
					Tentacle::TentacleTraverser overlayTraverser(speed_set.getTentacle(longest_index));
					visualizeTentacle(2, longest_index);
					/*while(overlayTraverser.hasNext())
					{
						aero_path_planning::Point point = overlayTraverser.next();
						try
						{
							rendering.setPointTrait(point, aero_path_planning::TENTACLE);
						}catch(std::exception& e){
							ROS_ERROR("%s", e.what());
						}
					}
					//ROS_INFO("This Is The Occupancy Grid With Selected Tentacle Overlaid:\n%s", rendering.toString(0,0)->c_str());*/
					this->occupancy_buffer_.pop_front();
				}
				//Send the velocity command to the platform
				sendVelCom(current_radius, current_velocity);
			}
			//spin to let ROS process callbacks
			ros::spinOnce();
		}

	}
private:

	int		platform_;		///flat marking what platform we're running on
	bool	should_plan_;	///Flag for signaling if the local planner should be running

	double	goal_weight_;	///weighting factor to bias tentacle selection towards the goal point
	double	trav_weight_;	///weighting factor to bias tentacle selection away from previously traversed points
	double	diff_weight_;	///weighting factor to bias tentacle selection away from difficult terrain
	double  unkn_weight_;	///weighting factor to bias tentacle selection towards unknown terrain
	double	x_dim_;		///x dimension of the occupancy grid to use, in real units
	double	y_dim_;		///y dimension of the occupancy grid to use, in real units
	double	z_dim_;		///z dimension of the occupancy grid to use, in real units
	double	res_;		///resolution the occupancy grid to use, in real units per grid unit
	double	current_vel_;	///Current Velocity of the Platform
	double	current_rad_;	///Current Radius followed by the Platform
	ros::NodeHandle nh_;	///Node handle for publishing/subscribing to topics
	std::string	v_action_topic_;		///Actionlib topic name to send velocity commands over
	std::string pc_topic_;			///topic name of the ROS topic to receive new occupancy grid data over
	aero_path_planning::Point	origin_;	///The origin to use for the occupancy grids
	TentacleGeneratorPtr tentacles_;	///Pointer to the tentacle generator which contains the tentacles to use for planning
	ros::Subscriber 	pc_sub_;		///Subscriber to the ROS topic to receive new occupancy grid data over
	ros::Subscriber		stop_sub_;	///Subscriber to the ROS topic to receive the software stop message
	ros::Publisher		vel_pub_;	///Publisher for Twist messages to a platform that takes them
	ros::Publisher          tent_pub_;       ///Publisher for visualizing selected tentacles

	boost::circular_buffer<OccupancyGrid > occupancy_buffer_;	///Buffer to store received OccupancyGrid data

	actionlib::SimpleActionClient<aero_drive_controller::VelocityCommandAction> v_client_;	///The actionlib client to set velocity command messages to the base platform with

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for handling the SoftwareStop message
	 * @param message The message to process
	 */
	void stopCB(const oryx_msgs::SoftwareStopConstPtr& message){
		//Need to flip as message is true when should stop
		this->should_plan_ = !message->stop;
		ROS_WARN("Oryx Local Path Planner Received A Software Stop Message [%s]: %s",(message->stop)?"Stop":"Go", message->message.c_str());
		//If we were told to stop, clear the buffer so we don't process stale data at wakeup
		if(!this->should_plan_)this->occupancy_buffer_.clear();
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for processing new point cloud data
	 * @param message The aero_path_planning::OccupancyGridMsg message to process
	 *
	 * Takes the data from the PointCloud2 message, processes it into a new occupancy grid,
	 * and places it on the occupancy grid buffer for processing by the planner
	 */
	void pcCB(const aero_path_planning::OccupancyGridMsgConstPtr& message){
		//To prevent processing of stale data, ignore anything received while we shouldn't be planning
		if(this->should_plan_)
		{
			ROS_INFO("I Got new Occupancy Grid Data!");
			OccupancyGrid recievedGrid(*message);
			//ROS_INFO("Grid Processed...");
			//ROS_INFO("Callback got the grid:\n%s", recievedGrid.toString(0,0)->c_str());
			this->occupancy_buffer_.push_back(recievedGrid);
		}
	}

	/**
	 * Performs platform specific sending of velocity commands
	 * @param velocity The linear velocity in +x to follow
	 * @param radius The radius of curvature to follow
	 */
	void sendVelCom(double velocity, double radius)
	{
		switch (this->platform_)
		{
		case 0:
			send(velocity, radius);
			break;
		case 1:
			twist(velocity, velocity/radius);
			break;
		default:
			break;
		}
	}

	/**
	 * Sends out geometery_msgs::Twist messages to the platform
	 * @param x_dot The linear velocity in +x
	 * @param omega The angular velocity around +z
	 */
	void twist(double x_dot, double omega)
	{
		geometry_msgs::Twist message;
		message.linear.x  = x_dot;
		message.angular.z = omega;
		this->vel_pub_.publish(message);
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Sends a command to the base's velocity controller
	 * @param velocity	The linear velocity to follow
	 * @param radius	The arc-radius to follow
	 */
	void send(double velocity, double radius)
	{
		aero_drive_controller::VelocityCommandGoal newGoal;
		newGoal.velocity	= velocity;
		newGoal.radius		= radius;
		//Once again, have to used boost::bind because you are inside a class
		v_client_.sendGoal(newGoal, boost::bind(&LocalPlanner::doneCb, this, _1, _2),
				boost::bind(&LocalPlanner::activeCb, this),
				boost::bind(&LocalPlanner::feedbackCb, this, _1));
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for handling the result from the velocity controller
	 * @param state		State that the goal finished in
	 * @param result	The result message returned from the velocity controller
	 */
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const aero_drive_controller::VelocityCommandResultConstPtr& result)
	{
		ROS_DEBUG("Finished in state [%s]", state.toString().c_str());
		ROS_DEBUG("Result: %s", (result->success)?"Successful":"Unsuccessful");
	};

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for processing the response that a velocity goal went active
	 */
	void activeCb()
	{
		ROS_DEBUG("Platform Received Velocity Goal");
	};

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for handling velocity controller feedback
	 * @param feedback The feedback message that needs to be processed
	 */
	void feedbackCb(const aero_drive_controller::VelocityCommandFeedbackConstPtr& feedback)
	{
		this->current_vel_ = feedback->velocity;
		this->current_rad_ = feedback->omega;
	};
	
	void visualizeTentacle(int speed_set, int tentacle)
	{
	  sensor_msgs::PointCloud2 message;
	  pcl::toROSMsg(this->tentacles_->getSpeedSet(speed_set).getTentacle(tentacle).getPoints(), message);
	  message.header.frame_id = "/laser";
	  message.header.stamp    = ros::Time::now();
	  this->tent_pub_.publish(message);
	}

};

//*********************** NODE IMPLEMENTATION ******************************//

int main(int argc, char **argv) {
	ros::init(argc, argv, "aero_base_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");
	//Default Parameter Values
	//*****************Communication Parameters*******************//
	std::string v_com_top(VEL_CMD_TOPIC);
	//std::string t_com_top("translate_command_topic");
	std::string pc_top(OCCUPANCY_TOPIC);

	//*****************Configuration Parameters*******************//
	//The platform that the local planner is running on
	std::string p_platform(PLATFORM);
	int platform = 0;
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
	double x_dim = 200;
	std::string x_dim_msg("");
	x_dim_msg+= boost::lexical_cast<double>(x_dim);
	x_dim_msg+="m";

	//y dimension of occupancy grid
	std::string p_y_dim(L_OCC_YDIM);
	double y_dim = 200;
	std::string y_dim_msg("");
	y_dim_msg+= boost::lexical_cast<double>(y_dim);
	y_dim_msg+="m";

	//z dimension of occupancy grid
	std::string p_z_dim(L_OCC_ZDIM);
	double z_dim = 0;
	std::string z_dim_msg("");
	z_dim_msg+= boost::lexical_cast<double>(z_dim);
	z_dim_msg+="m";

	//resolution occupancy grid
	std::string p_res(L_OCC_RES);
	double res = .01;
	std::string p_res_msg("");
	p_res_msg+= boost::lexical_cast<double>(res);
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
	double y_ori = y_dim/2;
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

	//Node Information Printout
	ROS_INFO("Starting Up Oryx Local Planner Version %d.%d.%d", oryx_path_planner_VERSION_MAJOR, oryx_path_planner_VERSION_MINOR, oryx_path_planner_VERSION_BUILD);

	//Get Private Parameters
	if(!p_nh.getParam(p_platform, platform))
	{
		PARAM_WARN(p_platform, platform_message);
	}
	else
	{
		switch(platform)
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
	if(!p_nh.getParam(v_com_top,v_com_top))		PARAM_WARN(v_com_top,	v_com_top);
	if(!p_nh.getParam(pc_top,	pc_top))		PARAM_WARN(pc_top,		pc_top);
	if(!p_nh.getParam(p_goal_weight,	goal_weight))PARAM_WARN(p_goal_weight,p_goal_weight_msg);
	if(!p_nh.getParam(p_trav_weight,	trav_weight))PARAM_WARN(p_trav_weight,p_trav_weight_msg);
	if(!p_nh.getParam(p_diff_weight,	diff_weight))PARAM_WARN(p_diff_weight,p_diff_weight_msg);
	if(!p_nh.getParam(p_unkn_weight,	unkn_weight))PARAM_WARN(p_unkn_weight,p_unkn_weight_msg);
	//Get Public Parameters
	if(!nh.getParam(p_up_rate,	update_rate))	PARAM_WARN(p_up_rate,	up_rate_msg);
	if(!nh.getParam(p_x_dim,	x_dim))			PARAM_WARN(p_x_dim,		x_dim_msg);
	if(!nh.getParam(p_y_dim,	y_dim))			PARAM_WARN(p_y_dim,		y_dim_msg);
	if(!nh.getParam(p_z_dim,	z_dim))			PARAM_WARN(p_z_dim,		z_dim_msg);
	if(!nh.getParam(p_x_ori,	x_ori))			PARAM_WARN(p_x_ori,		p_x_ori_msg);
	if(!nh.getParam(p_y_ori,	y_ori))			PARAM_WARN(p_y_ori,		p_y_ori_msg);
	if(!nh.getParam(p_z_ori,	z_ori))			PARAM_WARN(p_z_ori,		p_z_ori_msg);
	if(!nh.getParam(p_res,		res))			PARAM_WARN(p_res,		p_res_msg);
	if(!nh.getParam(p_num_tent,	num_tent))		PARAM_WARN(p_num_tent,	p_num_tent_msg);
	if(!nh.getParam(p_exp_fact,	exp_fact))		PARAM_WARN(p_exp_fact,	p_exp_fact_msg);
	if(!nh.getParam(p_num_speed_set, num_speed_set))PARAM_WARN(p_num_speed_set,	p_numSpeedSet_msg);
	if(!nh.getParam(p_max_speed,	max_speed))		PARAM_WARN(p_max_speed,	p_max_speed_msg);
	if(!nh.getParam(p_min_speed,	min_speed))		PARAM_WARN(p_min_speed,	p_min_speed_msg);

	//Set up Tentacles
	ROS_INFO("Generating Tentacles...");
	boost::shared_ptr<TentacleGenerator> tentacle_ptr(new aero_path_planning::TentacleGenerator (min_speed, max_speed, num_speed_set, num_tent, exp_fact, res, x_dim, y_dim/2));
	ROS_INFO("Tentacles Generated!");
	//Set up client to Drive Controller
	try
	{
		aero_path_planning::Point origin;
		origin.x=x_ori;
		origin.y=y_ori;
		origin.z=z_ori;
		PRINT_POINT("Origin Point", origin);
		LocalPlanner planner(platform, goal_weight, trav_weight, diff_weight, unkn_weight, x_dim, y_dim, z_dim, res, origin, v_com_top,  pc_top, tentacle_ptr);
		planner.doPlanning();
	}
	catch(std::exception& e)
	{
		ROS_FATAL("%s, %s",e.what(), "Shutting Down");
		ros::shutdown();
	}
}


//};
