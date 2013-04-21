#include "roboteq_driver/roboteq_manager_lib.h"

namespace roboteq_driver{

RoboteqManagerClient::RoboteqManagerClient(ros::NodeHandle& n, std::string control_topic){
  control_pub = n.advertise<RoboteqGroupMotorControl>(control_topic, 1000);
}

static RoboteqMotorControl createMotorControl(uint8_t channel, uint8_t control_mode, double setpoint){
  RoboteqMotorControl msg;
  msg.channel = channel;
  msg.control_mode = control_mode;
  msg.setpoint = setpoint;
  return msg;
}

void RoboteqManagerClient::setRPM(double chan1, double chan2){
  RoboteqGroupMotorControl msg;
  msg.motors.push_back(createMotorControl(1, RoboteqMotorControl::RPM, chan1));
  msg.motors.push_back(createMotorControl(2, RoboteqMotorControl::RPM, chan2));
  control_pub.publish(msg);
}

void RoboteqManagerClient::setPower(double chan1, double chan2){
  RoboteqGroupMotorControl msg;
  msg.motors.push_back(createMotorControl(1, RoboteqMotorControl::POWER, chan1));
  msg.motors.push_back(createMotorControl(2, RoboteqMotorControl::POWER, chan2));
  control_pub.publish(msg);
}

}
