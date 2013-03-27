#ifndef ROBOTEQ_MANAGER_LIB_H_
#define ROBOTEQ_MANAGER_LIB_H_


#include <stdlib.h>
#include <string>
#include <stdint.h>
#include <ros/ros.h>
#include "roboteq_driver/RoboteqGroupMotorControl.h"

namespace roboteq_driver{

class RoboteqManagerClient{
 public:
  RoboteqManagerClient(ros::NodeHandle& n, std::string control_topic);
  void setRPM(double chan1, double chan2);
  void setPower(double chan1, double chan2);
 private:
  ros::Publisher control_pub;
};
}

#endif //ROBOTEQ_MANAGER_LIB_H_
