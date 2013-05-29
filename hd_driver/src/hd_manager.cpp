#include "ros/ros.h"
#include "hd_driver/hd_motor_controller.h"
#include "hd_driver/HDConfig.h"
#include "device_driver_base/MotorFeedback.h"
#include "device_driver_base/SetMotorPositionAction.h"
#include "robot_base_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>
#include "device_driver_base/serial_port.h"
#include "device_driver_base/reconfigurable_device_driver.h"
#include "device_driver_base/SensorLevels.h"

using namespace device_driver;
using namespace device_driver_base;

class HDManager : public device_driver::ReconfigurableDeviceDriver<hd_driver::HDConfig>{
private:


	boost::mutex controller_mutex;
	const boost::shared_ptr<hd_driver::HDMotorController> controller;
	ReconfigurableAdvertisePtr feedback_pub;
	ReconfigurableTimerPtr feedback_timer;

	ReconfigurableActionServerPtr control_server;

	ReconfigurableSubscriberPtr pause_sub;
	bool is_paused;

	std::string reference_frame_;
	std::string port_;


public:
	void controlCallback(const SetMotorPositionGoalConstPtr & goal, boost::shared_ptr<actionlib::SimpleActionServer<SetMotorPositionAction> >& as_){
	        SetMotorPositionResult result;
		try{
			ROS_INFO("Setting position to: [%d]", goal->position);
			{
				boost::lock_guard<boost::mutex> lock(controller_mutex);
				if(!is_paused)
				  controller->set_position(goal->position, goal->max_velocity);
			}
			while(1){
				{
					boost::lock_guard<boost::mutex> lock(controller_mutex);
					if(is_paused || !(controller->get_status()&STATUS_TRAJECTORY_RUNNING))
						break;
				}
				usleep(100000);
			}
			{
				boost::lock_guard<boost::mutex> lock(controller_mutex);
				if(is_paused){
   				        ROS_WARN("Motor was paused when attempting to move");
				        as_->setAborted(result, "Could not set motor position, motor was paused");
					return;
				}
				if(controller->get_trajectory_status()&TRAJECTORY_MOVE_ABORTED){	
					as_->setAborted(result, "Move was aborted");
   				        ROS_WARN("Move was aborted");
					ROS_WARN("Status: %u\n", controller->get(hd_driver::HDMotorController::memory_bank_ram, hd_driver::HDMotorController::variable_status_register));
					ROS_WARN("Fault: %u\n", controller->get(hd_driver::HDMotorController::memory_bank_ram, hd_driver::HDMotorController::variable_fault_register));
					return;
				}
				as_->setSucceeded(result);
			}
		} catch(device_driver::Exception& e){
			ROS_WARN_STREAM("Error setting motor position: "<<e.what());
			as_->setAborted(result, "Exception thrown while setting position");
		}
	}


	virtual void reconfigureStopped(hd_driver::HDConfig& config){
		port_ = config.port;
	}
	virtual void reconfigureOpen(hd_driver::HDConfig& config){
		control_server->setName(config.control_service);
		feedback_pub->setTopic(config.info_topic);
		feedback_timer->setPeriod(ros::Duration(1/config.feedback_rate));
		pause_sub->setTopic(config.pause_topic);
		reference_frame_ = config.reference_frame;
	}
	virtual void reconfigureRunning(hd_driver::HDConfig& config){
	}


	void pauseCallback(const robot_base_msgs::SoftwareStop::ConstPtr& msg){
		{
			boost::lock_guard<boost::mutex> lock(controller_mutex);
			if(msg->stop)
				controller->set_state(hd_driver::HDMotorController::amplifier_disabled);
			//device is enabled in position set
			is_paused = msg->stop;
		}
	}

	void feedbackTimerCallback(const ros::TimerEvent& e){
		try{
			MotorFeedback msg;
			{
				boost::lock_guard<boost::mutex> lock(controller_mutex);
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = reference_frame_;
				msg.position = controller->get_position();
			}
			feedback_pub->publish(msg);
		} catch(device_driver::Exception& e){
			ROS_WARN_STREAM("Error reading motor info: "<<e.what());
		}
	}

	void openDevice(){
		ROS_INFO_STREAM("Opening device '"<<port_<<"'");
		controller->open(port_);
		controller->set_state(hd_driver::HDMotorController::amplifier_disabled);//disable the controller on startup
	}
	void closeDevice(){
		ROS_INFO_STREAM("Closing device '"<<port_<<"'");
		try{
		        controller->set_state(hd_driver::HDMotorController::amplifier_disabled);//try to disable controller
		} catch(...){}
		controller->close();
	}

	HDManager():controller(new hd_driver::HDMotorController()), is_paused(false), reference_frame_("/hd_reference_frame"), port_("/dev/ttyUSB2"){
		addDriverStateFunctions(device_driver_state::OPEN, &HDManager::openDevice, &HDManager::closeDevice, this);

		pause_sub = addReconfigurableSubscriber<robot_base_msgs::SoftwareStop>(device_driver_state::RUNNING, "/pause", 1000, boost::bind(&HDManager::pauseCallback, this, _1));
		feedback_pub = addReconfigurableAdvertise<MotorFeedback>(device_driver_state::RUNNING, "hd_info", 1000);
		feedback_timer = addReconfigurableTimer<HDManager>(device_driver_state::RUNNING, ros::Duration(0.1), &HDManager::feedbackTimerCallback, this);
		control_server = addReconfigurableActionServer<SetMotorPositionAction>(device_driver_state::RUNNING, "hd_control", boost::bind(&HDManager::controlCallback, this, _1, _2));
	}

};

int main(int argc, char **argv){
	ros::init(argc, argv, "hd_manager");

	ROS_INFO("Initializing HD device");

	HDManager manager;
	ROS_INFO("Initialization Complete");

	manager.spin();

	return 0;
}
