#include "ros/ros.h"
#include "hd_driver/hd_motor_controller.h"
#include "hd_driver/HDConfig.h"
#include "hd_driver/HDMotorInfo.h"
#include "hd_driver/SetPositionAction.h"
#include "aero_srr_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>
#include "device_driver_base/serial_port.h"
#include "device_driver_base/reconfigurable_device_driver.h"
#include "device_driver_base/SensorLevels.h"

using namespace device_driver;

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
	void controlCallback(const hd_driver::SetPositionGoalConstPtr & goal, boost::shared_ptr<actionlib::SimpleActionServer<hd_driver::SetPositionAction> >& as_){
		try{
			ROS_DEBUG("Setting position to: [%d]", goal->position);
			{
				boost::lock_guard<boost::mutex> lock(controller_mutex);
				if(!is_paused)
					controller->set_position(goal->position);
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
				if(is_paused)
					return;
				if(controller->get_trajectory_status()&TRAJECTORY_MOVE_ABORTED)
					return;
				hd_driver::SetPositionResult result;
				as_->setSucceeded(result);
			}
		} catch(device_driver::Exception& e){
			ROS_WARN_STREAM("Error setting motor position: "<<e.what());
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


	void pauseCallback(const aero_srr_msgs::SoftwareStop::ConstPtr& msg){
		{
			boost::lock_guard<boost::mutex> lock(controller_mutex);
			if(msg->stop)
				controller->set_state(hd_driver::HDMotorController::amplifier_disabled);
			is_paused = msg->stop;
		}
	}

	void feedbackTimerCallback(const ros::TimerEvent& e){
		try{
			hd_driver::HDMotorInfo msg;
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
	}
	void closeDevice(){
		ROS_INFO_STREAM("Closing device '"<<port_<<"'");
		controller->close();
	}

	HDManager():controller(new hd_driver::HDMotorController()), is_paused(false), reference_frame_("/hd_reference_frame"), port_("/dev/ttyUSB2"){
		addDriverStateFunctions(device_driver_state::OPEN, &HDManager::openDevice, &HDManager::closeDevice, this);

		pause_sub = addReconfigurableSubscriber<aero_srr_msgs::SoftwareStop>(device_driver_state::RUNNING, "/pause", 1000, boost::bind(&HDManager::pauseCallback, this, _1));
		feedback_pub = addReconfigurableAdvertise<hd_driver::HDMotorInfo>(device_driver_state::RUNNING, "hd_info", 1000);
		feedback_timer = addReconfigurableTimer<HDManager>(device_driver_state::RUNNING, ros::Duration(0.1), &HDManager::feedbackTimerCallback, this);
		control_server = addReconfigurableActionServer<hd_driver::SetPositionAction>(device_driver_state::RUNNING, "hd_control", boost::bind(&HDManager::controlCallback, this, _1, _2));
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
