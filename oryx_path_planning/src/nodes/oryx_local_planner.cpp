/**
 * @file	oryx_base_planner.cpp
 * @date	Oct 23, 2012
 * @author	Adam Panzica
 * @brief	Planning node for doing local planning and movement operations
 */

//*********************** SYSTEM DEPENDENCIES ************************************//
#include<actionlib/client/simple_action_client.h>
#include<oryx_drive_controller/VelocityCommandAction.h>
#include<boost/lexical_cast.hpp>
//*********************** LOCAL DEPENDENCIES ************************************//
#include "OryxPathPlanning.h"
#include "OryxPathPlannerConfig.h"

//*********************** MACROS ************************************//
///Macro for printing out warning messages if default parameters are used
#define PARAM_WARN(param,value) ROS_WARN(warn_message.c_str(), param.c_str(), value.c_str())

using namespace oryx_path_planning;

//namespace oryx_path_planning{
//*********************** HELPER CLASS DEFINITIONS ******************************//
/**
 * @author	Adam Panzica
 * @brief	Action Client for sending velocity commands to the base platform
 */
class VelocityClient{
public:
	VelocityClient(std::string action_topic):
	action_topic(action_topic),
	client(action_topic, true){
		ROS_INFO("Waiting For Connection To Server On <%s>...", this->action_topic.c_str());
		if(client.waitForServer(ros::Duration(5))){
			ROS_INFO("Connection Established");
		}else ROS_ERROR("Could Not Establish Connection To Server!");
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

//*********************** NODE IMPLEMENTATION ******************************//

int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_base_planner");
	ros::NodeHandle nh;
	//Default Parameter Values
	std::string warn_message("Parameter <%s> Not Set. Using Default Value <%s>");
	//*****************Communication Parameters*******************//
	std::string v_com_top("~/velocity_command_topic");
	//std::string t_com_top("translate_command_topic");
	std::string pc_top("~/occupancy_point_cloud_topic");

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

	//z dimension of occupancy grid
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
	if(!nh.getParam(v_com_top,	v_com_top))		PARAM_WARN(v_com_top,	v_com_top);
	//if(!nh.getParam(t_com_top, t_com_top))	PARAM_WARN(t_com_top,	t_com_top);
	if(!nh.getParam(pc_top,		pc_top))		PARAM_WARN(pc_top,		pc_top);
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
	oryx_path_planning::TentacleGenerator TentacleGen(minSpeed, maxSpeed, numSpeedSet, numTent, expFact, res, xDim, yDim);
	//Set up client to Drive Controller
	ROS_INFO("Setting Up Communications...");
	VelocityClient client(v_com_top);

}


//};
