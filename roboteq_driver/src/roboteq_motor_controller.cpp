#include "roboteq_driver/roboteq_motor_controller.h"
#include "device_driver_base/driver_util.h"
#include <stdio.h>

using namespace device_driver;

namespace roboteq_driver{

#define MISSING_VALUE -1024

RoboteqMotorController::RoboteqMotorController(double maxRPM1,
					       double maxRPM2,
					       int ppr1,
					       int ppr2):
	maxRPM1_(maxRPM1),
	maxRPM2_(maxRPM2),
	ppr1_(ppr1),
	ppr2_(ppr2),
	motor_mode1_(MOTOR_MODE_UNDEFINED),
	motor_mode2_(MOTOR_MODE_UNDEFINED){

}
RoboteqMotorController::~RoboteqMotorController(){
	close();
}


void RoboteqMotorController::open(std::string port){
	int status = device_.Connect(port);
	if(status != RQ_SUCCESS){
		if(status==RQ_UNRECOGNIZED_DEVICE){
			DRIVER_EXCEPT(Exception, "Error connecting to device. The device is not recognized.");
		}
		else if(status==RQ_UNRECOGNIZED_VERSION){
			DRIVER_EXCEPT(Exception, "Error connecting to device. Invalid device version.");
		}
		else if(status==RQ_ERR_OPEN_PORT){
			DRIVER_EXCEPT(Exception, "Error connecting to device. Error occurred while trying to open the communication port.");
		}
		else{
			DRIVER_EXCEPT(Exception, "Error connecting to device.");
		}
	}
	//Wait 10 ms before sending another command to device
	usleep(10*1000);

	//Configure the device
	//set encoders pulse per rotation
	setConfig(_EPPR, 1, ppr1_);
	setConfig(_EPPR, 2, ppr2_);
	//set encoders usage to feedback
	setConfig(_EMOD, 1, 18);
	setConfig(_EMOD, 2, 34);

	//TODO configure temp pins
	//setup analog inputs for temp (unused mode)
	setConfig(_AMOD, 1, 1);
	setConfig(_AMOD, 4, 1);

	//set motors max rpm
	setConfig(_MXRPM, 1, maxRPM1_);
	setConfig(_MXRPM, 2, maxRPM2_);


	//Initialize to initial values
	motor_mode1_ = motor_mode2_ = MOTOR_MODE_UNDEFINED;//make sure to reset local values
	setMotorMode(1, MOTOR_MODE_RPM);
	setMotorMode(2, MOTOR_MODE_RPM);
	
	saveToEEPROM();
	
	setPower(1, 0);
	setPower(2, 0);
}

void RoboteqMotorController::close(){
	device_.Disconnect();
}

void RoboteqMotorController::setCommand(int commandItem, int index, int value){
	int status = device_.SetCommand(commandItem, index, value);
	if(status!=RQ_SUCCESS){
		if(status==RQ_ERR_NOT_CONNECTED)
			DRIVER_EXCEPT(Exception, "The device is not connected (call open)");
		else if(status==RQ_ERR_TRANSMIT_FAILED)
			DRIVER_EXCEPT(Exception, "Error occurred while transmitting data to device.");
		else if(status==RQ_ERR_SERIAL_IO)
			DRIVER_EXCEPT(Exception, "Error occurred to serial communication.");
		else if(status==RQ_ERR_SERIAL_RECEIVE)
			DRIVER_EXCEPT(Exception, "Error occurred while transmitting data from device.");
		else if(status==RQ_INVALID_RESPONSE)
			DRIVER_EXCEPT(Exception, "Invalid response to the issued command.");
		else if(status==RQ_INVALID_COMMAND_ITEM)
			DRIVER_EXCEPT(Exception, "Invalid command item");
		else if(status==RQ_INDEX_OUT_RANGE)
			DRIVER_EXCEPT(Exception, "The item index is out of range.");
		else
			DRIVER_EXCEPT(Exception, "Failed to set device command.");
	}
}

void RoboteqMotorController::setCommand(int commandItem){
  setCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
}

void RoboteqMotorController::setConfig(int configItem, int index, int value){
	int status = device_.SetConfig(configItem, index, value);
	if(status!=RQ_SUCCESS){
		if(status==RQ_ERR_NOT_CONNECTED)
			DRIVER_EXCEPT(Exception, "The device is not connected (call open)");
		else if(status==RQ_ERR_TRANSMIT_FAILED)
			DRIVER_EXCEPT(Exception, "Error occurred while transmitting data to device.");
		else if(status==RQ_ERR_SERIAL_IO)
			DRIVER_EXCEPT(Exception, "Error occurred to serial communication.");
		else if(status==RQ_ERR_SERIAL_RECEIVE)
			DRIVER_EXCEPT(Exception, "Error occurred while transmitting data from device.");
		else if(status==RQ_INVALID_RESPONSE)
			DRIVER_EXCEPT(Exception, "Invalid response to the issued command.");
		else if(status==RQ_INVALID_COMMAND_ITEM)
			DRIVER_EXCEPT(Exception, "Invalid configuration item");
		else if(status==RQ_INDEX_OUT_RANGE)
			DRIVER_EXCEPT(Exception, "The item index is out of range.");
		else
			DRIVER_EXCEPT(Exception, "Failed to set device configuration.");
	}
}
void RoboteqMotorController::setConfig(int configItem, int value){
	int status = device_.SetConfig(configItem, value);
	if(status!=RQ_SUCCESS){
		if(status==RQ_ERR_NOT_CONNECTED)
			DRIVER_EXCEPT(Exception, "The device is not connected (call open)");
		else if(status==RQ_ERR_TRANSMIT_FAILED)
			DRIVER_EXCEPT(Exception, "Error occurred while transmitting data to device.");
		else if(status==RQ_ERR_SERIAL_IO)
			DRIVER_EXCEPT(Exception, "Error occurred to serial communication.");
		else if(status==RQ_ERR_SERIAL_RECEIVE)
			DRIVER_EXCEPT(Exception, "Error occurred while transmitting data from device.");
		else if(status==RQ_INVALID_RESPONSE)
			DRIVER_EXCEPT(Exception, "Invalid response to the issued command.");
		else if(status==RQ_INVALID_COMMAND_ITEM)
			DRIVER_EXCEPT(Exception, "Invalid configuration item");
		else if(status==RQ_INDEX_OUT_RANGE)
			DRIVER_EXCEPT(Exception, "The item index is out of range.");
		else
			DRIVER_EXCEPT(Exception, "Failed to set device configuration.");
	}
}
void RoboteqMotorController::getValue(int operatingItem, int index, int& value){
	int status = device_.GetValue(operatingItem, index, value);
	if(status!=RQ_SUCCESS){
		if(status==RQ_ERR_NOT_CONNECTED)
			DRIVER_EXCEPT(Exception, "The device is not connected (call open)");
		else if(status==RQ_ERR_TRANSMIT_FAILED)
			DRIVER_EXCEPT(Exception, "Error occurred while transmitting data to device.");
		else if(status==RQ_ERR_SERIAL_IO)
			DRIVER_EXCEPT(Exception, "Error occurred to serial communication.");
		else if(status==RQ_ERR_SERIAL_RECEIVE)
			DRIVER_EXCEPT(Exception, "Error occurred while transmitting data from device.");
		else if(status==RQ_INVALID_RESPONSE)
			DRIVER_EXCEPT(Exception, "Invalid response to the issued command.");
		else if(status==RQ_INVALID_COMMAND_ITEM)
			DRIVER_EXCEPT(Exception, "Invalid operating item");
		else if(status==RQ_INDEX_OUT_RANGE)
			DRIVER_EXCEPT(Exception, "The item index is out of range.");
		else
			DRIVER_EXCEPT(Exception, "Failed to get operating item value.");
	}
}



/*
 * Configuration
 */
void RoboteqMotorController::setSerialWatchdog(int time){
	setConfig(_RWD, time);
}

void RoboteqMotorController::setRotationInfo(double maxRPM1, double maxRPM2, int ppr1, int ppr2){
	maxRPM1_ = maxRPM1;
	maxRPM2_ = maxRPM2;
	ppr1_ = ppr1;
	ppr2_ = ppr2;
	setConfig(_EPPR, 1, ppr1_);
	setConfig(_EPPR, 2, ppr2_);
	setConfig(_MXRPM, 1, maxRPM1_);
	setConfig(_MXRPM, 2, maxRPM2_);

	saveToEEPROM();
}

void RoboteqMotorController::saveToEEPROM(){
  setCommand(_EES);
}


static inline double limit(double val, double min, double max){
  if(val<min)
    return min;
  if(val>max)
    return max;
  return val;
}
/*
 * Action
 */
void RoboteqMotorController::setMotorMode(uint8_t chan, MotorMode new_mode){
  if(chan==1){
    if(motor_mode1_!=new_mode){
      motor_mode1_ = new_mode;
      setConfig(_MMOD, 1, new_mode);
    }
  }
  else if(chan==2){
    if(motor_mode2_!=new_mode){
      motor_mode2_ = new_mode;
      setConfig(_MMOD, 2, new_mode);
    }
  }
  else
    DRIVER_EXCEPT(Exception, "Invalid motor channel");
}
void RoboteqMotorController::setRPM(uint8_t chan, double speed){
  setMotorMode(chan, MOTOR_MODE_RPM);
  if(chan==1)
    setCommand(_GO, 1, GO_COMMAND_BOUND*limit(speed, -maxRPM1_, maxRPM1_)/maxRPM1_);
  else if(chan==2)
    setCommand(_GO, 2, GO_COMMAND_BOUND*limit(speed, -maxRPM2_, maxRPM2_)/maxRPM2_);
  else
    DRIVER_EXCEPT(Exception, "Invalid motor channel");
}
void RoboteqMotorController::setPower(uint8_t chan, double power){
  setMotorMode(chan, MOTOR_MODE_POWER);
  if(chan==1)
    setCommand(_GO, 1, GO_COMMAND_BOUND*limit(power, -1, 1));
  else if(chan==2)
    setCommand(_GO, 2, GO_COMMAND_BOUND*limit(power, -1, 1));
  else
    DRIVER_EXCEPT(Exception, "Invalid motor channel");
}

/*
 * Sensors
 */
void RoboteqMotorController::getCurrent(uint8_t chan, double& value){
  int current;
  getValue(_MOTAMPS, chan, current);
  value = current*MOTOR_AMP_SCALE;
}
void RoboteqMotorController::getTemp(uint8_t chan, double& value){
  int an;
  if(chan==1)
    getValue(_AI, 1, an);
  else if(chan==2)
    getValue(_AI, 4, an);
  else
    DRIVER_EXCEPT(Exception, "Invalid motor channel");
  value = an*TEMP_SCALE+TEMP_OFFSET;
}
void RoboteqMotorController::getPosition(uint8_t chan, double& value){
  if(chan==1){
    int32_t raw_value;
    getValue(_C, chan, raw_value);
    value = ((double)raw_value)/ppr1_/counts_per_pulse;
  }
  else if(chan==2){
    int32_t raw_value;
    getValue(_C, chan, raw_value);
    value = ((double)raw_value)/ppr2_/counts_per_pulse;
  }
  else
    DRIVER_EXCEPT(Exception, "Invalid motor channel");
}
void RoboteqMotorController::getVelocity(uint8_t chan, double& value){
  if(chan==1){
    int32_t raw_value;
    getValue(_S, chan, raw_value);
    value = raw_value;//*maxRPM1_/GO_COMMAND_BOUND;
  }
  else if(chan==2){
    int32_t raw_value;
    getValue(_S, chan, raw_value);
    value = raw_value;//*maxRPM1_/GO_COMMAND_BOUND;
  }
  else
    DRIVER_EXCEPT(Exception, "Invalid motor channel");
}

}
