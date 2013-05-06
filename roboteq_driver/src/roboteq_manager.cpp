#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roboteq_driver/roboteq_motor_controller.h"
#include "roboteq_driver/RoboteqGroupInfo.h"
#include "roboteq_driver/RoboteqGroupMotorControl.h"
#include "aero_srr_msgs/SoftwareStop.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>
#include "device_driver_base/reconfigurable_device_driver.h"
#include "roboteq_driver/RoboteqConfig.h"

using namespace device_driver;

class RoboteqManager : public device_driver::ReconfigurableDeviceDriver<roboteq_driver::RoboteqConfig>{
private:
	boost::mutex controller_mutex;
	roboteq_driver::RoboteqMotorController controller;

	ReconfigurableAdvertisePtr feedback_pub;
	ReconfigurableTimerPtr feedback_timer;

	ReconfigurableThrottledSubscriberPtr control_sub;

	ReconfigurableSubscriberPtr pause_sub;

	bool is_paused_;

	std::string port_;
	double max_rpm_;
	int ppr_;

	void controlCallback(const roboteq_driver::RoboteqGroupMotorControl::ConstPtr& msg){
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		BOOST_FOREACH(roboteq_driver::RoboteqMotorControl motorControl, msg->motors){
			if(is_paused_)
				controller.setPower(motorControl.channel, 0);
			else if(motorControl.control_mode==roboteq_driver::RoboteqMotorControl::RPM)
				controller.setRPM(motorControl.channel, motorControl.setpoint);
			else if(motorControl.control_mode==roboteq_driver::RoboteqMotorControl::POWER)
				controller.setPower(motorControl.channel, motorControl.setpoint);
			else
				ROS_WARN("Control mode %d not implemented", motorControl.control_mode);
		}
	}
	void pauseCallback(const aero_srr_msgs::SoftwareStop::ConstPtr& msg){
		boost::lock_guard<boost::mutex> lock(controller_mutex);
		is_paused_ = msg->stop;
		controller.setPower(1, 0);
		controller.setPower(2, 0);
	}

	void getFeedback(uint8_t chan, roboteq_driver::RoboteqMotorInfo& chanFeedback){
		chanFeedback.channel = chan;

		controller.getCurrent(chan, chanFeedback.current);

		controller.getTemp(chan, chanFeedback.temp.temperature);
		chanFeedback.temp.header.stamp = ros::Time::now();

		controller.getPosition(chan, chanFeedback.position);

		controller.getVelocity(chan, chanFeedback.velocity);
	}
	void feedbackTimerCallback(const ros::TimerEvent& e){
		roboteq_driver::RoboteqGroupInfo feedback;
		roboteq_driver::RoboteqMotorInfo chan1Feedback;
		roboteq_driver::RoboteqMotorInfo chan2Feedback;
		{
			boost::lock_guard<boost::mutex> lock(controller_mutex);
			feedback.header.stamp = ros::Time::now();

			getFeedback(1, chan1Feedback);
			getFeedback(2, chan2Feedback);
		}

		feedback.motors.push_back(chan1Feedback);
		feedback.motors.push_back(chan2Feedback);

		feedback_pub->publish(feedback);
	}

	void openDevice(){
		ROS_INFO_STREAM("Opending device '"<<port_<<"'");
		controller.open(port_);
		controller.saveRotationInfo(max_rpm_, max_rpm_, ppr_, ppr_);
	}
	void closeDevice(){
		ROS_INFO_STREAM("Closing device '"<<port_<<"'");
		try{
			controller.setPower(0, 0);
		} catch(...){}
		controller.close();
	}
public:
	RoboteqManager():is_paused_(false), port_("/dev/ttyACM0"), max_rpm_(250), ppr_(250), controller(max_rpm_, max_rpm_, ppr_, ppr_){
		addDriverStateFunctions(device_driver_state::OPEN, &RoboteqManager::openDevice, &RoboteqManager::closeDevice, this);

		control_sub = addReconfigurableThrottledSubscriber<roboteq_driver::RoboteqGroupMotorControl>(device_driver_state::RUNNING, 10, "roboteq_control", 1000, boost::bind(&RoboteqManager::controlCallback, this, _1));
		pause_sub = addReconfigurableSubscriber<aero_srr_msgs::SoftwareStop>(device_driver_state::RUNNING, "/pause", 1000, boost::bind(&RoboteqManager::pauseCallback, this, _1));
		feedback_pub = addReconfigurableAdvertise<roboteq_driver::RoboteqGroupInfo>(device_driver_state::RUNNING, "roboteq_info", 1000);
		feedback_timer = addReconfigurableTimer<RoboteqManager>(device_driver_state::RUNNING, ros::Duration(0.1), &RoboteqManager::feedbackTimerCallback, this);
	}

	virtual void reconfigureStopped(roboteq_driver::RoboteqConfig& config){
		port_ = config.port;
		max_rpm_ = config.max_rpm;
		ppr_ = config.ppr;
	}
	virtual void reconfigureOpen(roboteq_driver::RoboteqConfig& config){
		control_sub->setMaxRate(config.control_rate);
		control_sub->setTopic(config.control_topic);

		feedback_timer->setPeriod(ros::Duration(1/config.feedback_rate));
		feedback_pub->setTopic(config.info_topic);

		pause_sub->setTopic(config.pause_topic);
	}
	virtual void reconfigureRunning(roboteq_driver::RoboteqConfig& config){
	}

};

int main(int argc, char **argv){
  ros::init(argc, argv, "roboteq_manager");
  
  RoboteqManager manager;
  manager.spin();

  ROS_INFO("Initializing Roboteq Device");

  return 0;
}
