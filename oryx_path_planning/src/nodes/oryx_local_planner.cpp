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
//*********************** LOCAL DEPENDENCIES ************************************//
#include "OryxPathPlanning.h"
#include "OryxPathPlannerConfig.h"

//*********************** MACROS ************************************//
///Macro for printing out warning messages if default parameters are used
#define PARAM_WARN(param,value) ROS_WARN(warn_message.c_str(), param.c_str(), value.c_str())

///Number of seconds to wait for connections to other ROS nodes before determining a system failure
#define CONNECTION_TIMEOUT	5.0

using namespace oryx_path_planning;

//namespace oryx_path_planning{
//*********************** HELPER CLASS DEFINITIONS ******************************//
/**
 * @author	Adam Panzica
 * @brief	Action Client for sending velocity commands to the base platform
 */
class VelocityClient{
public:
	VelocityClient(std::string action_topic) throw(std::runtime_error):
		action_topic(action_topic),
		client(action_topic, true){
		ROS_INFO("Waiting For Connection To Server On <%s>...", this->action_topic.c_str());
		if(client.waitForServer(ros::Duration(CONNECTION_TIMEOUT))){
			ROS_INFO("Connection Established");
		}else{
			ROS_ERROR("Could Not Establish Connection To Server!");
			std::string errMsg("Unable to connect to velocity command server over ");
			throw std::runtime_error(errMsg+this->action_topic);
		}
	}
	virtual ~VelocityClient(){}

	//Called when the goal is completed
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const oryx_drive_controller::VelocityCommandResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Result: %s", (result->success)?"Successful":"Unsuccessful");
	};

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active...");
	};

	// Called every time feedback is received for the goal
	void feedbackCb(const oryx_drive_controller::VelocityCommandFeedbackConstPtr& feedback)
	{
		ROS_INFO("Feedback: <%f,%f>", feedback->velocity, feedback->omega);
	};

	//Send a goal to the server
	void send(double velocity, double radius){
		oryx_drive_controller::VelocityCommandGoal newGoal;
		newGoal.velocity	= velocity;
		newGoal.radius		= radius;
		//Once again, have to used boost::bind because you are inside a class
		client.sendGoal(newGoal, boost::bind(&VelocityClient::doneCb, this, _1, _2),
				boost::bind(&VelocityClient::activeCb, this),
				boost::bind(&VelocityClient::feedbackCb, this, _1));
	}
private:
	ros::NodeHandle	nh;
	std::string		action_topic;
	actionlib::SimpleActionClient<oryx_drive_controller::VelocityCommandAction> client;
};

/**
 * @author	Adam Panzica
 * @brief	Class which does the actual work of local path planning and sending commands to base
 */
class LocalPlanner{
public:
	LocalPlanner(double xDim, double yDim, double zDim, double res, std::string& oc_point_cloud_topic, boost::shared_ptr<TentacleGenerator> tentacles, boost::shared_ptr<VelocityClient> v_client):
		v_client(v_client),
		tentacles(tentacles),
		point_cloud_topic(oc_point_cloud_topic),
		occupancy_buffer(2){
		this->xDim = xDim;
		this->yDim = yDim;
		this->zDim = zDim;
		this->res  = res;
		this->pc_sub = nh.subscribe(this->point_cloud_topic, 1, &LocalPlanner::pcCB, this);
		this->shouldPlan = true;
	}
	virtual ~LocalPlanner(){};

	/**
	 * Callback for processing new point cloud data
	 * @param message
	 */
	void pcCB(const sensor_msgs::PointCloud2ConstPtr& message){
		ROS_INFO("I Got new Occupancy Grid Data!");
		PointCloudPtr cloud;
		pcl::fromROSMsg(*message, *cloud);
		this->occupancy_buffer.push_back(OccupancyGridPtr(new OccupancyGrid(xDim, yDim, zDim, res, cloud )));
	}

	/**
	 * does the actual work of performing path planning
	 */
	void doPlanning(){
		while(ros::ok()){
			if(shouldPlan){
				//Grab the next occupancy grid to process
				if(!this->occupancy_buffer.empty()){
					boost::shared_ptr<OccupancyGrid> workingGrid_ptr = this->occupancy_buffer.front();
					this->occupancy_buffer.pop_front();

					ROS_INFO("I'm Processing The Following Occupancy Grid:\n%s", workingGrid_ptr.get()->toString(0,0).get()->c_str());
					for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator itr = workingGrid_ptr->getGrid()->begin(); itr<workingGrid_ptr->getGrid()->end(); itr++){
						if(itr->rgba!=oryx_path_planning::UNKNOWN||itr->rgba!=0){
							ROS_INFO("Found a Point <%f,%f,%f> with RGBA = %d", itr->x, itr->y, itr->z, itr->rgba);
						}
					}
				}
			}
			//spin to let ROS process callbacks
			ros::spinOnce();
		}
	}
private:
	bool	shouldPlan;
	double	xDim;
	double	yDim;
	double	zDim;
	double	res;
	ros::NodeHandle nh;
	boost::shared_ptr<VelocityClient> v_client;
	boost::shared_ptr<TentacleGenerator> tentacles;
	std::string point_cloud_topic;
	ros::Subscriber pc_sub;

	boost::circular_buffer<boost::shared_ptr<OccupancyGrid> > occupancy_buffer;

};

//*********************** NODE IMPLEMENTATION ******************************//

int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_base_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");
	//Default Parameter Values
	std::string warn_message("Parameter <%s> Not Set. Using Default Value <%s>");
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

	//Node Information Printout
	ROS_INFO("Starting Up Oryx Base Planner Version %d.%d.%d", oryx_path_planner_VERSION_MAJOR, oryx_path_planner_VERSION_MINOR, oryx_path_planner_VERSION_BUILD);

	//Get Private Parameters
	if(!p_nh.getParam(v_com_top,v_com_top))		PARAM_WARN(v_com_top,	v_com_top);
	//if(!nh.getParam(t_com_top, t_com_top))	PARAM_WARN(t_com_top,	t_com_top);
	if(!p_nh.getParam(pc_top,	pc_top))		PARAM_WARN(pc_top,		pc_top);
	if(!nh.getParam(p_up_rate,	update_rate))	PARAM_WARN(p_up_rate,	up_rate_msg);
	if(!nh.getParam(p_x_dim,	xDim))			PARAM_WARN(p_x_dim,		xDim_msg);
	if(!nh.getParam(p_y_dim,	yDim))			PARAM_WARN(p_y_dim,		yDim_msg);
	if(!nh.getParam(p_z_dim,	zDim))			PARAM_WARN(p_z_dim,		zDim_msg);
	if(!nh.getParam(p_res,		res))			PARAM_WARN(p_res,		p_res_msg);
	if(!nh.getParam(p_numTent,	numTent))		PARAM_WARN(p_numTent,	p_numTent_msg);
	if(!nh.getParam(p_expFact,	expFact))		PARAM_WARN(p_expFact,	p_expFact_msg);
	if(!nh.getParam(p_numSpeedSet, numSpeedSet))PARAM_WARN(p_numSpeedSet,	p_numSpeedSet_msg);
	if(!nh.getParam(p_maxSpeed,	maxSpeed))		PARAM_WARN(p_maxSpeed,	p_maxSpeed_msg);
	if(!nh.getParam(p_minSpeed,	minSpeed))		PARAM_WARN(p_minSpeed,	p_minSpeed_msg);

	//Set up Tentacles
	ROS_INFO("Generating Tentacles...");
	boost::shared_ptr<TentacleGenerator> tentacle_ptr(new oryx_path_planning::TentacleGenerator (minSpeed, maxSpeed, numSpeedSet, numTent, expFact, res, xDim, yDim));
	ROS_INFO("Tentacles Generated!");
	//Set up client to Drive Controller
	ROS_INFO("Setting Up Communication With Base...");
	try{
		boost::shared_ptr<VelocityClient> v_client_ptr(new VelocityClient(v_com_top));
		ROS_INFO("Communications Established!");
		boost::shared_ptr<LocalPlanner> l_client_ptr(new LocalPlanner(xDim, yDim, zDim, res, pc_top, tentacle_ptr, v_client_ptr));
		l_client_ptr->doPlanning();
	}catch(std::exception& e){
		ROS_FATAL("%s, %s",e.what(), "Shutting Down");
		ros::shutdown();
	}
}


//};
