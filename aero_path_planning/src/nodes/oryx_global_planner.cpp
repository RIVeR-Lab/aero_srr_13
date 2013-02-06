/**
 * @file	oryx_global_planner.cpp
 * @date	Jan 1, 2013
 * @author	Adam Panzica
 * @brief	//TODO fill in detailed discription here
 */

//*********************SYSTEM DEPENDENCIES**********************//
#include<ros/ros.h>
#include<boost/function.hpp>
#include<boost/unordered_map.hpp>
#include<aero_srr_msgs/Command.h>

//*********************LOCAL DEPENDENCIES**********************//
#include"OryxPathPlanning.h"
#include"OryxPathPlannerConfig.h"


using namespace aero_path_planning;

/**
 * @brief Command processing engine for the global planner
 * @author Adam Panzica
 * @details A class defining a series of commands, and their respective callbacks, that allows the the global planner to communicate
 * with a client node. Its intention is to allow for the control of the global planner via remote connection during runtime,
 * for debugging and safety reasons.
 */
class CommandEngine
{
private:
	/**
	 * Enum defining the levels of criticality for command functions
	 */
	typedef enum command_criticality_
	{
		FATAL,   //!< FATAL Failure to process the command is a fatal error
		CRITICAL,//!< CRITICAL Failure to process the command is a critical error
		WARNING  //!< WARNING Failure to process the command is only a warning
	} command_criticality;

	typedef boost::function<bool (std::string& data, aero_srr_msgs::Command::Response& response)> command_func;	///Typedef for the signature of command functions
	typedef boost::unordered_map<std::string, std::pair<command_func, command_criticality> > command_map;///Typedef for a map of command functions to their command keyword
public:
	CommandEngine()
{
		init();
		bindCommands();
}
	CommandEngine(ros::NodeHandle& nh):
		nh_(nh)
	{
		init();
		bindCommands();
	}
	virtual ~CommandEngine(){};
private:
	ros::NodeHandle nh_;		///Node handle for communicating with ROS system
	ros::ServiceServer comm_srv_; ///Service Server for processing Command services

	/**
	 * Mapping of command functions to their string keyword. Each command has a integer criticality associated with it,
	 * which can be used to judge the consequences of commands failing
	 */
	command_map com_map_;

	/**
	 * Perform common initialization
	 */
	void init()
	{
		this->comm_srv_ = this->nh_.advertiseService("/global_planning/commands", &CommandEngine::commandCB, this);
	}
	/**
	 * Registers all of the commands with their processing callback
	 */
	void bindCommands()
	{
		//Command keywords
		std::string start_com("start");
		std::string stop_com("stop");
		std::string manual_com("manual");
		std::string auto_com("autonomous");
		//Bind keywords to their callback
		this->com_map_[start_com].first  = boost::bind(&CommandEngine::startCB, this, _1, _2);
		this->com_map_[start_com].second = FATAL;

		this->com_map_[stop_com].first   =boost::bind(&CommandEngine::stopCB, this, _1, _2);
		this->com_map_[stop_com].second  = FATAL;

		this->com_map_[manual_com].first =boost::bind(&CommandEngine::manCB, this, _1, _2);
		this->com_map_[manual_com].second= FATAL;

		this->com_map_[auto_com].first   =boost::bind(&CommandEngine::autoCB, this, _1, _2);
		this->com_map_[auto_com].second  = FATAL;
	}

	/**
	 * Callback for processing received commands
	 * @author Adam Panzica
	 * @param request The service request
	 * @param response The service response
	 */
	bool commandCB(aero_srr_msgs::Command::Request& request, aero_srr_msgs::Command::Response& response)
	{
		std::string command_message("Could Not Process The Command: ");
		//Check to see if the command is registered
		if(this->com_map_.count(request.command)==1)
		{
			//Try to process the command
			if(!this->com_map_[request.command].first(request.data, response))
			{
				//If it failed, handle the consequences
				switch(this->com_map_[request.command].second)
				{
				case FATAL:
					ROS_FATAL_STREAM(command_message<<request.command<<": "<<response.data);
					break;
				case CRITICAL:
					ROS_ERROR_STREAM(command_message<<request.command<<": "<<response.data);
					break;
				case WARNING:
					ROS_WARN_STREAM(command_message<<request.command<<": "<<response.data);
					break;
				default:
					break;
				}
			}
			else
			{
				response.success = true;
				return true;
			}
		}
		else
		{
			ROS_ERROR_STREAM("Received Invalid Command: "<<request.command);
			response.success = false;
			return false;
		}
		return false;
	}

	/**
	 * Callback for processing the "start" command
	 * @author Adam Panzica
	 * @param data Command data to be parsed
	 * @return TRUE if the command successfully processed, else FALSE
	 *
	 * This command starts the global planner into whatever mode it was in last (or default if there was no mode specified)
	 */
	bool startCB(std::string& data, aero_srr_msgs::Command::Response& response)
	{
		return false;
	}

	/**
	 * Callback for processing the "start" command
	 * @author Adam Panzica
	 * @param data Command data to be parsed
	 * @return TRUE if the command successfully processed, else FALSE
	 *
	 * This command stops the global planner from running, except to process new commands from the CommandEngine
	 */
	bool stopCB(std::string& data, aero_srr_msgs::Command::Response& response)
	{
		return false;
	}


	/**
	 * Callback for processing the "manual" command
	 * @author Adam Panzica
	 * @param data Command data to be parsed
	 * @return TRUE if the command successfully processed, else FALSE
	 *
	 * This command puts the global planner into Manual Control mode. It lets it know to expect Joy Messages to control it
	 */
	bool manCB(std::string& data, aero_srr_msgs::Command::Response& response)
	{
		return false;
	}

	/**
	 * Callback for processing the "autonomous" command
	 * @author Adam Panzica
	 * @param data Command data to be parsed
	 * @return TRUE if the command successfully processed, else FALSE
	 *
	 * This command puts the global planner into Autonomous Control mode.
	 */
	bool autoCB(std::string& data, aero_srr_msgs::Command::Response& response)
	{
		return false;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aero_global_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");


	//Comunication Parameters
	std::string local_planner_topic("occupancy_point_cloud_topic");
	std::string odometry_topic("odometry_topic");
	std::string command_topic("/global_planning/commands");

	//Configuration Parameters
	//*****************Configuration Parameters*******************//
	//Minimum update rate expected of occupancy grid
	std::string p_up_rate("occupancy/update_rate");
	double update_rate = 0.2;
	std::string up_rate_msg("");
	up_rate_msg+= boost::lexical_cast<double>(update_rate);
	up_rate_msg+="s";

	//x dimension of occupancy grid
	std::string p_x_dim("occupancy/x_dimension");
	double x_dim = 200;
	std::string x_dim_msg("");
	x_dim_msg+= boost::lexical_cast<double>(x_dim);
	x_dim_msg+="m";

	//y dimension of occupancy grid
	std::string p_y_dim("occupancy/y_dimension");
	double y_dim = 200;
	std::string y_dim_msg("");
	y_dim_msg+= boost::lexical_cast<double>(y_dim);
	y_dim_msg+="m";

	//z dimension of occupancy grid
	std::string p_z_dim("occupancy/z_dimension");
	double z_dim = 0;
	std::string z_dim_msg("");
	z_dim_msg+= boost::lexical_cast<double>(z_dim);
	z_dim_msg+="m";

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
	double y_ori = y_dim/2;
	std::string p_y_ori_msg("");
	p_y_ori_msg+= boost::lexical_cast<double>(y_ori);
	p_y_ori_msg+="m";

	//Node Information Printout
	ROS_INFO("Starting Up Oryx Global Planner Version %d.%d.%d", oryx_path_planner_VERSION_MAJOR, oryx_path_planner_VERSION_MINOR, oryx_path_planner_VERSION_BUILD);

	//Get Private Parameters
	if(!p_nh.getParam(local_planner_topic,local_planner_topic))	PARAM_WARN(local_planner_topic,	local_planner_topic);
	if(!p_nh.getParam(odometry_topic,	odometry_topic))		PARAM_WARN(odometry_topic,		odometry_topic);

	//Get Public Parameters
	if(!nh.getParam(p_up_rate,	update_rate))	PARAM_WARN(p_up_rate,	up_rate_msg);
	if(!nh.getParam(p_x_dim,	x_dim))			PARAM_WARN(p_x_dim,		x_dim_msg);
	if(!nh.getParam(p_y_dim,	y_dim))			PARAM_WARN(p_y_dim,		y_dim_msg);
	if(!nh.getParam(p_z_dim,	z_dim))			PARAM_WARN(p_z_dim,		z_dim_msg);
	if(!nh.getParam(p_x_ori,	x_ori))			PARAM_WARN(p_x_ori,		p_x_ori_msg);
	if(!nh.getParam(p_y_ori,	y_ori))			PARAM_WARN(p_y_ori,		p_y_ori_msg);
	if(!nh.getParam(p_z_ori,	z_ori))			PARAM_WARN(p_z_ori,		p_z_ori_msg);
	if(!nh.getParam(p_res,		res))			PARAM_WARN(p_res,		p_res_msg);

	ROS_INFO("Global Planner Configuration Parameters Set...");


}
