#include "roboteq_driver/roboteq_motor_controller.h"

namespace roboteq_driver{

RoboteqMotorController::RoboteqMotorController(double rotations_per_meter1,
					       double rotations_per_meter2,
					       double maxMPS1,
					       double maxMPS2,
					       int ppr1,
					       int ppr2):
	rotations_per_meter1_(rotations_per_meter1),
	rotations_per_meter2_(rotations_per_meter2),
	maxMPS1_(maxMPS1),
	maxMPS2_(maxMPS2),
	ppr1_(ppr1),
	ppr2_(ppr2){

}
RoboteqMotorController::~RoboteqMotorController(){
	close();
}


void RoboteqMotorController::open(std::string port){
	int status = device_.Connect(port);
	if(status != RQ_SUCCESS){
		if(status==RQ_UNRECOGNIZED_DEVICE){
			throw Exception("Error connecting to device. The device is not recognized.");
		}
		else if(status==RQ_UNRECOGNIZED_VERSION){
			throw Exception("Error connecting to device. Invalid device version.");
		}
		else if(status==RQ_ERR_OPEN_PORT){
			throw Exception("Error connecting to device. Error occurred while trying to open the communication port.");
		}
		else{
			throw Exception("Error connecting to device.");
		}
	}
	//Wait 10 ms before sending another command to device
	sleepms(10);

	//Configure the device
	//set encoders pulse per rotation
	setConfig(_EPPR, 1, ppr1_);
	setConfig(_EPPR, 2, ppr2_);
	//set encoders usage to feedback
	setConfig(_EMOD, 1, 1+16);
	setConfig(_EMOD, 2, 1+32);

	//setup analog inputs for temp (unused mode)
	setConfig(_AINA, 1, 0);
	setConfig(_AINA, 2, 0);

	//set motors max rpm
	setConfig(_MXRPM, 1, maxMPS1_*rotations_per_meter1_);
	setConfig(_MXRPM, 2, maxMPS2_*rotations_per_meter2_);
	//set motors mode to closed-loop speed
	setConfig(_MMOD, 1, 2);
	setConfig(_MMOD, 2, 2);
}

void RoboteqMotorController::close(){
	device_.Disconnect();
}

void RoboteqMotorController::setCommand(int commandItem, int index, int value){
	int status = device_.SetCommand(commandItem, index, value);
	if(status!=RQ_SUCCESS){
		if(status==RQ_ERR_NOT_CONNECTED)
			throw Exception("The device is not connected (call open)");
		else if(status==RQ_ERR_TRANSMIT_FAILED)
			throw Exception("Error occurred while transmitting data to device.");
		else if(status==RQ_ERR_SERIAL_IO)
			throw Exception("Error occurred to serial communication.");
		else if(status==RQ_ERR_SERIAL_RECEIVE)
			throw Exception("Error occurred while transmitting data from device.");
		else if(status==RQ_INVALID_RESPONSE)
			throw Exception("Invalid response to the issued command.");
		else if(status==RQ_INVALID_COMMAND_ITEM)
			throw Exception("Invalid command item");
		else if(status==RQ_INDEX_OUT_RANGE)
			throw Exception("The item index is out of range.");
		else
			throw Exception("Failed to set device command.");
	}
}
void RoboteqMotorController::setConfig(int configItem, int index, int value){
	int status = device_.SetConfig(configItem, index, value);
	if(status!=RQ_SUCCESS){
		if(status==RQ_ERR_NOT_CONNECTED)
			throw Exception("The device is not connected (call open)");
		else if(status==RQ_ERR_TRANSMIT_FAILED)
			throw Exception("Error occurred while transmitting data to device.");
		else if(status==RQ_ERR_SERIAL_IO)
			throw Exception("Error occurred to serial communication.");
		else if(status==RQ_ERR_SERIAL_RECEIVE)
			throw Exception("Error occurred while transmitting data from device.");
		else if(status==RQ_INVALID_RESPONSE)
			throw Exception("Invalid response to the issued command.");
		else if(status==RQ_INVALID_COMMAND_ITEM)
			throw Exception("Invalid configuration item");
		else if(status==RQ_INDEX_OUT_RANGE)
			throw Exception("The item index is out of range.");
		else
			throw Exception("Failed to set device configuration.");
	}
}
void RoboteqMotorController::setConfig(int configItem, int value){
	int status = device_.SetConfig(configItem, value);
	if(status!=RQ_SUCCESS){
		if(status==RQ_ERR_NOT_CONNECTED)
			throw Exception("The device is not connected (call open)");
		else if(status==RQ_ERR_TRANSMIT_FAILED)
			throw Exception("Error occurred while transmitting data to device.");
		else if(status==RQ_ERR_SERIAL_IO)
			throw Exception("Error occurred to serial communication.");
		else if(status==RQ_ERR_SERIAL_RECEIVE)
			throw Exception("Error occurred while transmitting data from device.");
		else if(status==RQ_INVALID_RESPONSE)
			throw Exception("Invalid response to the issued command.");
		else if(status==RQ_INVALID_COMMAND_ITEM)
			throw Exception("Invalid configuration item");
		else if(status==RQ_INDEX_OUT_RANGE)
			throw Exception("The item index is out of range.");
		else
			throw Exception("Failed to set device configuration.");
	}
}
void RoboteqMotorController::getValue(int operatingItem, int index, int& value){
	int status = device_.GetConfig(operatingItem, index, value);
	if(status!=RQ_SUCCESS){
		if(status==RQ_ERR_NOT_CONNECTED)
			throw Exception("The device is not connected (call open)");
		else if(status==RQ_ERR_TRANSMIT_FAILED)
			throw Exception("Error occurred while transmitting data to device.");
		else if(status==RQ_ERR_SERIAL_IO)
			throw Exception("Error occurred to serial communication.");
		else if(status==RQ_ERR_SERIAL_RECEIVE)
			throw Exception("Error occurred while transmitting data from device.");
		else if(status==RQ_INVALID_RESPONSE)
			throw Exception("Invalid response to the issued command.");
		else if(status==RQ_INVALID_COMMAND_ITEM)
			throw Exception("Invalid operating item");
		else if(status==RQ_INDEX_OUT_RANGE)
			throw Exception("The item index is out of range.");
		else
			throw Exception("Failed to get operating item value.");
	}
}



/*
 * Configuration
 */
void RoboteqMotorController::setAccel(double chan1, double chan2){
	setCommand(_ACCEL, 1, chan1*rotations_per_meter1_);
	setCommand(_ACCEL, 2, chan2*rotations_per_meter2_);
}
void RoboteqMotorController::setDecel(double chan1, double chan2){
	setCommand(_DECEL, 1, chan1*rotations_per_meter1_);
	setCommand(_DECEL, 2, chan2*rotations_per_meter2_);
}
void RoboteqMotorController::setSerialWatchdog(int time){
	setConfig(_RWD, time);
}

/*
 * Action
 */
void RoboteqMotorController::setSpeed(double chan1, double chan2){
	setCommand(_GO, 1, GO_COMMAND_BOUND*chan1/maxMPS1_);
	setCommand(_GO, 2, GO_COMMAND_BOUND*chan2/maxMPS2_);
}

/*
 * Sensors
 */
void RoboteqMotorController::getCurrent(double& chan1, double& chan2){
	int current1, current2;
	getValue(_MOTAMPS, 1, current1);
	getValue(_MOTAMPS, 2, current2);
	chan1 = current1*MOTOR_AMP_SCALE;
	chan2 = current2*MOTOR_AMP_SCALE;
}
void RoboteqMotorController::getTemp(double& chan1, double& chan2){
	int an1, an2;
	getValue(_ANAIN, 1, an1);
	getValue(_ANAIN, 2, an2);
	chan1 = an1*TEMP_SCALE+TEMP_OFFSET;
	chan2 = an2*TEMP_SCALE+TEMP_OFFSET;
}

}
