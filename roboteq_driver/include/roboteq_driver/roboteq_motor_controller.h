#ifndef ROBOTEQ_MOTOR_CONTROLLER_H_
#define ROBOTEQ_MOTOR_CONTROLLER_H_

#include <stdlib.h>
#include <string>
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"
#include <exception>
#include <stdexcept>

namespace roboteq_driver{

class Exception : public std::runtime_error{
 public:
 Exception(const std::string& msg):std::runtime_error(msg){}
};

class RoboteqMotorController{
 private:
	const static int GO_COMMAND_BOUND = 1000;
	const static double MOTOR_AMP_SCALE = 10;
	/**
	 * degree C/mV
	 */
	const static double TEMP_SCALE = 1;
	/**
	 * degree C
	 */
	const static double TEMP_OFFSET = 0;
 private:
	void setCommand(int commandItem, int index, int value);
	void setConfig(int configItem, int index, int value);
	void setConfig(int configItem, int value);
	void getValue(int operatingItem, int index, int& value);
 public:
	/**
	 * @param rotations_per_meter1 the rotations per meter for motor 1
	 * @param rotations_per_meter2 the rotations per meter for motor 2
	 * @param maxMPS1 the maximum meters per second for motor 1
	 * @param maxMPS2 the maximum meters per second for motor 2
	 * @param ppr1 the pulses per rotation for encoder 1 (motor 1)
	 * @param ppr2 the pulses per rotation for encoder 2 (motor 2)
	 */
	RoboteqMotorController(double rotations_per_meter1, double rotations_per_meter2,
			       double maxMPS1, double maxMPS2,
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
	/**
	 * set the acceleration of the motors in m/s^2
	 */
	void setAccel(double chan1, double chan2);
	/**
	 * set the deceleration of the motors in m/s^2
	 */
	void setDecel(double chan1, double chan2);
	/**
	 * set the timeout for the serial connection (0 to disable)
	 */
	void setSerialWatchdog(int time);

	/*
	 * Action
	 */
	/**
	 * set the speed of each motor (in m/s)
	 */
	void setSpeed(double chan1, double chan2);

	/*
	 * Sensors
	 */
	/**
	 * get the current draw of each motor (in A)
	 */
	void getCurrent(double& chan1, double& chan2);
	/**
	 * get the temperature of each motor (in C)
	 */
	void getTemp(double& chan1, double& chan2);
 private:
	const double rotations_per_meter1_;
	const double rotations_per_meter2_;
	const double maxMPS1_;
	const double maxMPS2_;
	const int ppr1_;
	const int ppr2_;
	RoboteqDevice device_;
};

}

#endif //ROBOTEQ_MOTOR_CONTROLLER_H_
