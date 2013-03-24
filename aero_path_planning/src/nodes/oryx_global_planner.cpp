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
#include<aero_path_planning/OryxPathPlanning.h>
#include"OryxPathPlannerConfig.h"
#include<aero_path_planning/GlobalPlanner.h>
#include<aero_path_planning/RRTCarrot.h>


using namespace aero_path_planning;

///**
// * @brief Command processing engine for the global planner
// * @author Adam Panzica
// * @details A class defining a series of commands, and their respective callbacks, that allows the the global planner to communicate
// * with a client node. Its intention is to allow for the control of the global planner via remote connection during runtime,
// * for debugging and safety reasons.
// */
//class CommandEngine
//{
//private:
//	/**
//	 * Enum defining the levels of criticality for command functions
//	 */
//	typedef enum command_criticality_
//	{
//		FATAL,   //!< FATAL Failure to process the command is a fatal error
//		CRITICAL,//!< CRITICAL Failure to process the command is a critical error
//		WARNING  //!< WARNING Failure to process the command is only a warning
//	} command_criticality;
//
//	typedef boost::function<bool (std::string& data, aero_srr_msgs::Command::Response& response)> command_func;	///Typedef for the signature of command functions
//	typedef boost::unordered_map<std::string, std::pair<command_func, command_criticality> > command_map;///Typedef for a map of command functions to their command keyword
//public:
//	CommandEngine()
//{
//		init();
//		bindCommands();
//}
//	CommandEngine(ros::NodeHandle& nh):
//		nh_(nh)
//	{
//		init();
//		bindCommands();
//	}
//	virtual ~CommandEngine(){};
//private:
//	ros::NodeHandle nh_;		///Node handle for communicating with ROS system
//	ros::ServiceServer comm_srv_; ///Service Server for processing Command services
//
//	/**
//	 * Mapping of command functions to their string keyword. Each command has a integer criticality associated with it,
//	 * which can be used to judge the consequences of commands failing
//	 */
//	command_map com_map_;
//
//	/**
//	 * Perform common initialization
//	 */
//	void init()
//	{
//		this->comm_srv_ = this->nh_.advertiseService("/global_planning/commands", &CommandEngine::commandCB, this);
//	}
//	/**
//	 * Registers all of the commands with their processing callback
//	 */
//	void bindCommands()
//	{
//		//Command keywords
//		std::string start_com("start");
//		std::string stop_com("stop");
//		std::string manual_com("manual");
//		std::string auto_com("autonomous");
//		//Bind keywords to their callback
//		this->com_map_[start_com].first  = boost::bind(&CommandEngine::startCB, this, _1, _2);
//		this->com_map_[start_com].second = FATAL;
//
//		this->com_map_[stop_com].first   =boost::bind(&CommandEngine::stopCB, this, _1, _2);
//		this->com_map_[stop_com].second  = FATAL;
//
//		this->com_map_[manual_com].first =boost::bind(&CommandEngine::manCB, this, _1, _2);
//		this->com_map_[manual_com].second= FATAL;
//
//		this->com_map_[auto_com].first   =boost::bind(&CommandEngine::autoCB, this, _1, _2);
//		this->com_map_[auto_com].second  = FATAL;
//	}
//
//	/**
//	 * Callback for processing received commands
//	 * @author Adam Panzica
//	 * @param request The service request
//	 * @param response The service response
//	 */
//	bool commandCB(aero_srr_msgs::Command::Request& request, aero_srr_msgs::Command::Response& response)
//	{
//		std::string command_message("Could Not Process The Command: ");
//		//Check to see if the command is registered
//		if(this->com_map_.count(request.command)==1)
//		{
//			//Try to process the command
//			if(!this->com_map_[request.command].first(request.data, response))
//			{
//				//If it failed, handle the consequences
//				switch(this->com_map_[request.command].second)
//				{
//				case FATAL:
//					ROS_FATAL_STREAM(command_message<<request.command<<": "<<response.data);
//					break;
//				case CRITICAL:
//					ROS_ERROR_STREAM(command_message<<request.command<<": "<<response.data);
//					break;
//				case WARNING:
//					ROS_WARN_STREAM(command_message<<request.command<<": "<<response.data);
//					break;
//				default:
//					break;
//				}
//			}
//			else
//			{
//				response.success = true;
//				return true;
//			}
//		}
//		else
//		{
//			ROS_ERROR_STREAM("Received Invalid Command: "<<request.command);
//			response.success = false;
//			return false;
//		}
//		return false;
//	}
//
//	/**
//	 * Callback for processing the "start" command
//	 * @author Adam Panzica
//	 * @param data Command data to be parsed
//	 * @return TRUE if the command successfully processed, else FALSE
//	 *
//	 * This command starts the global planner into whatever mode it was in last (or default if there was no mode specified)
//	 */
//	bool startCB(std::string& data, aero_srr_msgs::Command::Response& response)
//	{
//		return false;
//	}
//
//	/**
//	 * Callback for processing the "start" command
//	 * @author Adam Panzica
//	 * @param data Command data to be parsed
//	 * @return TRUE if the command successfully processed, else FALSE
//	 *
//	 * This command stops the global planner from running, except to process new commands from the CommandEngine
//	 */
//	bool stopCB(std::string& data, aero_srr_msgs::Command::Response& response)
//	{
//		return false;
//	}
//
//
//	/**
//	 * Callback for processing the "manual" command
//	 * @author Adam Panzica
//	 * @param data Command data to be parsed
//	 * @return TRUE if the command successfully processed, else FALSE
//	 *
//	 * This command puts the global planner into Manual Control mode. It lets it know to expect Joy Messages to control it
//	 */
//	bool manCB(std::string& data, aero_srr_msgs::Command::Response& response)
//	{
//		return false;
//	}
//
//	/**
//	 * Callback for processing the "autonomous" command
//	 * @author Adam Panzica
//	 * @param data Command data to be parsed
//	 * @return TRUE if the command successfully processed, else FALSE
//	 *
//	 * This command puts the global planner into Autonomous Control mode.
//	 */
//	bool autoCB(std::string& data, aero_srr_msgs::Command::Response& response)
//	{
//		return false;
//	}
//};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aero_global_planner");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	ROS_INFO("Global Planner Configuration Parameters Set...");
	aero_path_planning::RRTCarrot path(1);
	aero_path_planning::GlobalPlanner planner(nh, p_nh, path);
	ros::spin();
}
