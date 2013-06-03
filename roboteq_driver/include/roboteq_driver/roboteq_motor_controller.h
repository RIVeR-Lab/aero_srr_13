#ifndef ROBOTEQ_MOTOR_CONTROLLER_H_
#define ROBOTEQ_MOTOR_CONTROLLER_H_

#include <stdlib.h>
#include <string>
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"
#include <exception>
#include <stdexcept>
#include <stdint.h>

namespace roboteq_driver{

class RoboteqMotorController{
 public:
  /**
   * a set of the supported motor modes
   */
  typedef int8_t MotorMode;
  const static MotorMode MOTOR_MODE_UNDEFINED = -1;
  const static MotorMode MOTOR_MODE_POWER = 0;
  const static MotorMode MOTOR_MODE_RPM = 1;
  const static MotorMode MOTOR_MODE_POSITION_REL = 2;
  const static MotorMode MOTOR_MODE_POSITION_COUNT = 3;
  const static MotorMode MOTOR_MODE_POSITION_TRACKING = 4;
  const static MotorMode MOTOR_MODE_TORQUE = 5;

  const static int32_t counts_per_pulse = 4;

 private:
  /**
   * The max bound of the go command
   */
  const static int GO_COMMAND_BOUND = 1000;
  /**
   * The scale for the raw amp values returned from the motor controller
   */
  const static double MOTOR_AMP_SCALE = 0.1;
  /**
   * degree C/mV
   */
  const static double TEMP_SCALE = 1;
  /**
   * degree C
   */
  const static double TEMP_OFFSET = 0;
 private:
  /**
   * execute a set command
   */
  void setCommand(int commandItem, int index, int value);
  /**
   * execute a set command
   */
  void setCommand(int commandItem);
  /**
   * execute a set config
   */
  void setConfig(int configItem, int index, int value);
  /**
   * execute a set config
   */
  void setConfig(int configItem, int value);
  /**
   * get a value
   */
  void getValue(int operatingItem, int index, int& value);
 public:
  /**
   * @param ppr1 the pulses per rotation for encoder 1 (motor 1)
   * @param ppr2 the pulses per rotation for encoder 2 (motor 2)
   */
  RoboteqMotorController(double maxRPM1, double maxRPM2,
			 int ppr1, int ppr2);
  ~RoboteqMotorController();
  /**
   * Open the connection to the motor controller
   */
  void open(std::string port);
  /**
   * Close the connection to the motor controller
   */
  void close();

  /*
   * Configuration
   */
  void setSerialWatchdog(int time);

  void saveRotationInfo(double maxRPM1, double maxRPM2,
			 int ppr1, int ppr2);
  /**
   * trigger_delay in ms
   * trigger_level in A
   */
  void setCurrentTrigger(uint8_t chan, uint32_t trigger_delay, float trigger_level);

  /**
   * Save the current configuration to persist between restarts
   */
  void saveToEEPROM();

  /*
   * Action
   */
  void setMotorMode(uint8_t chan, MotorMode new_mode);
  /**
   * set the speed of a motor (in rad/s)
   */
  void setRPM(uint8_t chan, double speed);
  /**
   * set the power of a motor (-1.0 to 1.0)
   */
  void setPower(uint8_t chan, double power);

  /*
   * Sensors
   */
  /**
   * get the current draw of each motor (in A)
   */
  void getCurrent(uint8_t chan, double& value);
  /**
   * get the temperature of each motor (in C)
   */
  void getTemp(uint8_t chan, double& value);
  /**
   * get the absolute position of each motor
   */
  void getPosition(uint8_t chan, double& value);
  /**
   * get the velocity of each motor
   */
  void getVelocity(uint8_t chan, double& value);
 private:
  double maxRPM1_;
  double maxRPM2_;
  int ppr1_;
  int ppr2_;
  RoboteqDevice device_;
  MotorMode motor_mode1_;
  MotorMode motor_mode2_;
};

}

#endif //ROBOTEQ_MOTOR_CONTROLLER_H_
