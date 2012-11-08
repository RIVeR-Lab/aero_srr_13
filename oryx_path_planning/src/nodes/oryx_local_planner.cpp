/**
 * @file	oryx_base_planner.cpp
 * @date	Oct 23, 2012
 * @author	Adam Panzica
 * @brief	Planning node for doing local planning and movement operations
 */

//*********************** SYSTEM DEPENDENCIES ************************************//
#include<queue>
#include<actionlib/client/simple_action_client.h>
#include<oryx_drive_controller/VelocityCommandAction.h>
#include<boost/lexical_cast.hpp>
#include<boost/circular_buffer.hpp>
#include<oryxsrr_msgs/SoftwareStop.h>
//*********************** LOCAL DEPENDENCIES ************************************//
#include "OryxPathPlanning.h"
#include "OryxPathPlannerConfig.h"

//*********************** MACROS ************************************//
///Number of seconds to wait for connections to other ROS nodes before determining a system failure
#define CONNECTION_TIMEOUT	5.0

using namespace oryx_path_planning;

//*********************** HELPER CLASS DEFINITIONS ******************************//

/**
 * @author	Adam Panzica
 * @brief	Class which does the actual work of local path planning and sending commands to base
 */
class LocalPlanner{
public:
	/**
	 * @author	Adam Panzica
	 * @brief	Creates a new local planner
	 * @param goalWeight			weighting factor to bias tentacle selection towards the goal point
	 * @param travWeight			weighting factor to bias tentacle selection away from previously traversed points
	 * @param diffWeight			weighting factor to bias tentacle selection away from difficult terrain
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
	LocalPlanner(double goalWeight, double travWeight, double diffWeight, double xDim, double yDim, double zDim, double res, oryx_path_planning::Point& origin, std::string& v_action_topic, std::string& oc_point_cloud_topic, TentacleGeneratorPtr tentacles) throw(std::runtime_error):
		v_action_topic(v_action_topic),
		pc_topic(oc_point_cloud_topic),
		origin(origin),
		tentacles(tentacles),
		occupancy_buffer(2),
		v_client(v_action_topic, true){
		this->goalWeight = goalWeight;
		this->travWeight = travWeight;
		this->diffWeight = diffWeight;
		this->xDim = xDim;
		this->yDim = yDim;
		this->zDim = zDim;
		this->res  = res;
		this->pc_sub = nh.subscribe(this->pc_topic, 1, &LocalPlanner::pcCB, this);
		this->shouldPlan = true;
		this->currentRad = 0;
		this->currentVel = 0;

		std::string software_stop_topic("oryx/software_stop");

		if(!this->nh.getParam(software_stop_topic, software_stop_topic))ROS_WARN("%s not set, using default value %s", software_stop_topic.c_str(), software_stop_topic.c_str());
		this->stop_sub = nh.subscribe(software_stop_topic, 1, &LocalPlanner::stopCB, this);

		ROS_INFO("Waiting For Connection To Velocity Control Server On <%s>...", this->v_action_topic.c_str());
		if(v_client.waitForServer(ros::Duration(CONNECTION_TIMEOUT))){
			ROS_INFO("Connection Established");
		}else{
			ROS_ERROR("Could Not Establish Connection To Velocity Control Server!");
			std::string errMsg("Unable to connect to velocity command server over ");
			throw std::runtime_error(errMsg+this->v_action_topic);
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
	void doPlanning(){
		while(ros::ok()){
			if(shouldPlan){
				//Grab the next occupancy grid to process
				if(!this->occupancy_buffer.empty()){
					OccupancyGrid	workingGrid= this->occupancy_buffer.front();
					SpeedSet	  	speedSet   = this->tentacles->getSpeedSet(this->currentVel);

					ROS_INFO("I'm Processing The Following Occupancy Grid:\n%s", workingGrid.toString(0,0).get()->c_str());
					int numPoints = 0;
					for(OccupancyGrid::iterator test_itr = workingGrid.begin(); test_itr<workingGrid.end(); test_itr++){
						//ROS_INFO_COND(test_itr->rgba != oryx_path_planning::FREE_LOW_COST, "Found an obsticale point! <%f, %f>", test_itr->x, test_itr->y);
						if(test_itr->rgba != oryx_path_planning::FREE_LOW_COST)numPoints++;
					}
					ROS_INFO("I Found %d number of interesting points", numPoints);



					ROS_INFO("I'm Going To Use A Speed Set With The Parameters <NumTent:%d, Vel:%f, SR:%f>", speedSet.getNumTentacle(), speedSet.getVelocity(), speedSet.getSeedRad());
					OccupancyGrid rendering(workingGrid);
					unsigned int longestIndex = 0;
					double longestLength = 0;
					double lengthModifier = 0;
					ROS_INFO("I'm looking for the longest tentacle...");
					for(unsigned int i=0; i<speedSet.getNumTentacle(); i++){
						bool traversing = true;
						Tentacle workingTentacle = speedSet.getTentacle(i);
						Tentacle::TentacleTraverser traverser(workingTentacle);
						lengthModifier = 0;

						while(traverser.hasNext()&&traversing){
							const oryx_path_planning::Point& point = traverser.next();
							try{
								switch(workingGrid.getPointTrait(point)){
								case oryx_path_planning::OBSTACLE:
									ROS_INFO("Hit Obstacle On Tentacle %d at length %f", i, traverser.lengthTraversed());
									PRINT_POINT("Hit Point", point);
									traversing = false;
									break;
								case oryx_path_planning::GOAL:
									ROS_INFO("Hit the Goal on Tentacle %d at length %f", i, traverser.lengthTraversed());
									traversing = false;
									break;
								case oryx_path_planning::FREE_HIGH_COST:
									lengthModifier -= traverser.deltaLength()*this->diffWeight;
									break;
								case oryx_path_planning::TRAVERSED:
									lengthModifier -= traverser.deltaLength()*this->travWeight;
									break;
								default:
									break;
								}
							}catch(OccupancyGridAccessException& e){
								ROS_ERROR("%s", e.what());
							}

						}

						//Check to see if the current tentacle is the best one
						if(traverser.lengthTraversed() >= (longestLength+lengthModifier)){
							longestIndex = i;
							longestLength = traverser.lengthTraversed();
							//if traversing was set to false, we hit the goal, break out of the tentacle search
							if(!traversing) break;
						}
					}
					//Print out the selected tentacle on the grid
					ROS_INFO("I Selected Tentacle %d, length %f", longestIndex, longestLength);
					Tentacle::TentacleTraverser overlayTraverser(speedSet.getTentacle(longestIndex));
					while(overlayTraverser.hasNext()){
						oryx_path_planning::Point point = overlayTraverser.next();
						try{
							rendering.setPointTrait(point, oryx_path_planning::TENTACLE);
						}catch(std::exception& e){
							ROS_ERROR("%s", e.what());
						}
					}
					ROS_INFO("This Is The Occupancy Grid With Selected Tentacle Overlaid:\n%s", rendering.toString(0,0)->c_str());
					this->occupancy_buffer.pop_front();
				}
			}
			//spin to let ROS process callbacks
			ros::spinOnce();
		}

	}
private:

	bool	shouldPlan;	///Flag for signaling if the local planner should be running

	double	goalWeight;	///weighting factor to bias tentacle selection towards the goal point
	double	travWeight;	///weighting factor to bias tentacle selection away from previously traversed points
	double	diffWeight;	///weighting factor to bias tentacle selection away from difficult terrain
	double	xDim;		///x dimension of the occupancy grid to use, in real units
	double	yDim;		///y dimension of the occupancy grid to use, in real units
	double	zDim;		///z dimension of the occupancy grid to use, in real units
	double	res;		///resolution the occupancy grid to use, in real units per grid unit
	double	currentVel;	///Current Velocity of the Platform
	double	currentRad;	///Current Radius followed by the Platform
	ros::NodeHandle nh;	///Node handle for publishing/subscribing to topics
	std::string	v_action_topic;		///Actionlib topic name to send velocity commands over
	std::string pc_topic;			///topic name of the ROS topic to receive new occupancy grid data over
	oryx_path_planning::Point	origin;	///The origin to use for the occupancy grids
	TentacleGeneratorPtr tentacles;	///Pointer to the tentacle generator which contains the tentacles to use for planning
	ros::Subscriber 	pc_sub;		///Subscriber to the ROS topic to receive new occupancy grid data over
	ros::Subscriber		stop_sub;	///Subscriber to the ROS topic to receive the software stop message

	boost::circular_buffer<OccupancyGrid > occupancy_buffer;	///Buffer to store received OccupancyGrid data

	actionlib::SimpleActionClient<oryx_drive_controller::VelocityCommandAction> v_client;	///The actionlib client to set velocity command messages to the base platform with

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for handling the SoftwareStop message
	 * @param message The message to process
	 */
	void stopCB(const oryxsrr_msgs::SoftwareStopConstPtr& message){
		//Need to flip as message is true when should stop
		this->shouldPlan = !message->stop;
		ROS_WARN("Oryx Local Path Planner Received A Software Stop Message [%s]: %s",(message->stop)?"Stop":"Go", message->message.c_str());
		//If we were told to stop, clear the buffer so we don't process stale data at wakeup
		if(!this->shouldPlan)this->occupancy_buffer.clear();
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Callback for processing new point cloud data
	 * @param message The sensor_msgs::PointCloud2 message to process
	 *
	 * Takes the data from the PointCloud2 message, processes it into a new occupancy grid,
	 * and places it on the occupancy grid buffer for processing by the planner
	 */
	void pcCB(const sensor_msgs::PointCloud2ConstPtr& message){
		//To prevent processing of stale data, ignore anything received while we shouldn't be planning
		if(this->shouldPlan){
			ROS_INFO("I Got new Occupancy Grid Data!");
			OccupancyGridCloud cloud;
			pcl::fromROSMsg<pcl::PointXYZRGBA>(*message, cloud);
			OccupancyGrid recievedGrid(xDim, yDim, zDim, res, this->origin, cloud);
			ROS_INFO("Grid Processed...");
			ROS_INFO("Callback got the grid:\n%s", recievedGrid.toString(0,0)->c_str());
			this->occupancy_buffer.push_back(recievedGrid);
		}
	}

	/**
	 * @author	Adam Panzica
	 * @brief	Sends a command to the base's velocity controller
	 * @param velocity	The linear velocity to follow
	 * @param radius	The arc-radius to follow
	 */
	void send(double velocity, double radius){
		oryx_drive_controller::VelocityCommandGoal newGoal;
		newGoal.velocity	= velocity;
		newGoal.radius		= radius;
		//Once again, have to used boost::bind because you are inside a class
		v_client.sendGoal(newGoal, boost::bind(&LocalPlanner::doneCb, this, _1, _2),
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
			const oryx_drive_controller::VelocityCommandResultConstPtr& result)
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
	void feedbackCb(const oryx_drive_controller::VelocityCommandFeedbackConstPtr& feedback)
	{
		this->currentVel = feedback->velocity;
		this->currentRad = feedback->omega;
	};

};

//*********************** NODE IMPLEMENTATION ******************************//

int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_base_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");
	//Default Parameter Values
	//*****************Communication Parameters*******************//
	std::string v_com_top("velocity_command_topic");
	//std::string t_com_top("translate_command_topic");
	std::string pc_top("occupancy_point_cloud_topic");

	//*****************Configuration Parameters*******************//
	//Minimum update rate expected of occupancy grid
	std::string p_up_rate("occupancy/update_rate");
	double update_rate = 0.2;
	std::string up_rate_msg("");
	up_rate_msg+= boost::lexical_cast<double>(update_rate);
	up_rate_msg+="s";

	//x dimension of occupancy grid
	std::string p_x_dim("occupancy/x_dimension");
	double xDim = 200;
	std::string xDim_msg("");
	xDim_msg+= boost::lexical_cast<double>(xDim);
	xDim_msg+="m";

	//y dimension of occupancy grid
	std::string p_y_dim("occupancy/y_dimension");
	double yDim = 200;
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
	double y_ori = yDim/2;
	std::string p_y_ori_msg("");
	p_y_ori_msg+= boost::lexical_cast<double>(y_ori);
	p_y_ori_msg+="m";


	//number of tentacles per speed set
	std::string p_numTent("tentacles/number");
	int numTent = 81;
	std::string p_numTent_msg("");
	p_numTent_msg+= boost::lexical_cast<double>(numTent);
	p_numTent_msg+=" Tentacles";

	//Exponential Factor to use for generating seed radii
	std::string p_expFact("tentacles/exp_factor");
	double expFact = 1.15;
	std::string p_expFact_msg("");
	p_numTent_msg+= boost::lexical_cast<double>(expFact);
	p_numTent_msg+=" Tentacles";

	//number of tentacles per speed set
	std::string p_numSpeedSet("speed_set/number");
	int numSpeedSet = 15;
	std::string p_numSpeedSet_msg("");
	p_numSpeedSet_msg+= boost::lexical_cast<double>(numSpeedSet);
	p_numSpeedSet_msg+= " Speed Sets";

	//Max Speed
	std::string p_maxSpeed("speed_set/max_speed");
	double maxSpeed = 1;
	std::string p_maxSpeed_msg("");
	p_maxSpeed_msg+= boost::lexical_cast<double>(maxSpeed);
	p_maxSpeed_msg+="m/s";

	//Min Speed
	std::string p_minSpeed("speed_set/min_speed");
	double minSpeed = 1;
	std::string p_minSpeed_msg("");
	p_minSpeed_msg+= boost::lexical_cast<double>(minSpeed);
	p_minSpeed_msg+="m/s";

	//Goal Weight
	std::string p_goalWeight("goal_weight");
	double goalWeight = 2;
	std::string p_goalWeight_msg("");
	p_goalWeight_msg+= boost::lexical_cast<double>(goalWeight);

	//Traversed Weight
	std::string p_travWeight("traversed_weight");
	double travWeight = 0.1;
	std::string p_travWeight_msg("");
	p_travWeight_msg+= boost::lexical_cast<double>(travWeight);

	//Difficulty Weight
	std::string p_diffWeight("difficult_weight");
	double diffWeight = 0.1;
	std::string p_diffWeight_msg("");
	p_diffWeight_msg+= boost::lexical_cast<double>(diffWeight);

	//Node Information Printout
	ROS_INFO("Starting Up Oryx Base Planner Version %d.%d.%d", oryx_path_planner_VERSION_MAJOR, oryx_path_planner_VERSION_MINOR, oryx_path_planner_VERSION_BUILD);

	//Get Private Parameters
	if(!p_nh.getParam(v_com_top,v_com_top))		PARAM_WARN(v_com_top,	v_com_top);
	if(!p_nh.getParam(pc_top,	pc_top))		PARAM_WARN(pc_top,		pc_top);
	if(!p_nh.getParam(p_goalWeight,	goalWeight))PARAM_WARN(p_goalWeight,p_goalWeight_msg);
	if(!p_nh.getParam(p_travWeight,	travWeight))PARAM_WARN(p_travWeight,p_travWeight_msg);
	if(!p_nh.getParam(p_diffWeight,	diffWeight))PARAM_WARN(p_diffWeight,p_diffWeight_msg);
	//Get Public Parameters
	if(!nh.getParam(p_up_rate,	update_rate))	PARAM_WARN(p_up_rate,	up_rate_msg);
	if(!nh.getParam(p_x_dim,	xDim))			PARAM_WARN(p_x_dim,		xDim_msg);
	if(!nh.getParam(p_y_dim,	yDim))			PARAM_WARN(p_y_dim,		yDim_msg);
	if(!nh.getParam(p_z_dim,	zDim))			PARAM_WARN(p_z_dim,		zDim_msg);
	if(!nh.getParam(p_x_ori,	x_ori))			PARAM_WARN(p_x_ori,		p_x_ori_msg);
	if(!nh.getParam(p_y_ori,	y_ori))			PARAM_WARN(p_y_ori,		p_y_ori_msg);
	if(!nh.getParam(p_z_ori,	z_ori))			PARAM_WARN(p_z_ori,		p_z_ori_msg);
	if(!nh.getParam(p_res,		res))			PARAM_WARN(p_res,		p_res_msg);
	if(!nh.getParam(p_numTent,	numTent))		PARAM_WARN(p_numTent,	p_numTent_msg);
	if(!nh.getParam(p_expFact,	expFact))		PARAM_WARN(p_expFact,	p_expFact_msg);
	if(!nh.getParam(p_numSpeedSet, numSpeedSet))PARAM_WARN(p_numSpeedSet,	p_numSpeedSet_msg);
	if(!nh.getParam(p_maxSpeed,	maxSpeed))		PARAM_WARN(p_maxSpeed,	p_maxSpeed_msg);
	if(!nh.getParam(p_minSpeed,	minSpeed))		PARAM_WARN(p_minSpeed,	p_minSpeed_msg);

	//Set up Tentacles
	ROS_INFO("Generating Tentacles...");
	boost::shared_ptr<TentacleGenerator> tentacle_ptr(new oryx_path_planning::TentacleGenerator (minSpeed, maxSpeed, numSpeedSet, numTent, expFact, res, xDim, yDim/2));
	ROS_INFO("Tentacles Generated!");
	//Set up client to Drive Controller
	try{
		oryx_path_planning::Point origin;
		origin.x=x_ori;
		origin.y=y_ori;
		origin.z=z_ori;
		PRINT_POINT("Origin Point", origin);
		LocalPlanner planner(goalWeight, travWeight, diffWeight, xDim, yDim, zDim, res, origin, v_com_top,  pc_top, tentacle_ptr);
		planner.doPlanning();
	}catch(std::exception& e){
		ROS_FATAL("%s, %s",e.what(), "Shutting Down");
		ros::shutdown();
	}
}


//};
