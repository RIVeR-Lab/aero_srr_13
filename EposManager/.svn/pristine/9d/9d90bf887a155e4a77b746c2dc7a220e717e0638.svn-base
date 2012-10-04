/*
 * EPOS.h
 *
 *  Created on: Jan 20, 2012
 *      Author: oryx
 */

#ifndef EPOS_H_
#define EPOS_H_
#include "EposManager/Definitions.h"
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <EposManager/EposManagerConfig.h>


class EPOS {
public:

	EPOS(ros::NodeHandle _nh);

	/**
	 * Returns the Node ID of the EPOS Controller in a CAN chain.
	 *
	 * Returns the Node ID of the EPOS Controller. This node id is passed in as a parameter to the
	 * EPOS at runtime. The node ID for each EPOS can be configured through the DIP switches on the
	 * side of the EPOS.
	 *
	 * @return nodeID The node ID of the EPOS Controller
	 */
	int getNodeID();

	/**
	 * Returns the name of the Motor
	 *
	 * Returns the name of the motor. This name is passed in as a parameter at runtime. it is not
	 * essential, as it is only used in MotorInfo messages. It is primarily for the benefit of the
	 * user, so they can easily tell which motor specific data is coming from, rather than having to
	 * use only the node ID;
	 *
	 * @return motorName The name given to the motor
	 */
	std::string getMotorName();

	/**
	 * Initializes the motor to use the parameters passed in at runtime
	 *
	 * This method initializes the given EPOS using the various parameters passed into it at
	 * runtime. It is important that each of the parameters has an acceptable value, otherwise the
	 * initialization will fail. If any part of the initialization fails, it will stop the process
	 * and alert the user of the issue. It is also important that a valid keyhandle be passed into
	 * this method. If the keyhandle is not initialized beforehand, this will fail.
	 *
	 * @param keyHandle the Keyhandle for the EPOS Chain
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool initializeMotor(void* keyHandle);

	/**
	 * Returns an error message if a function fails.
	 *
	 * This method is called when a method fails for one reason or another. A string is passed to it
	 * containing information on the location of the failure (such as when trying to set or get the
	 * the velocity). The method then takes this string and appends the error code containing the cause
	 * behind the failure ( such as a USB Read Error). This is useful for debugging the code, or notifying
	 * other programs that there was an error in this program.
	 *
	 * @param error An error message stating where the program failed
	 * @return error A modified error message containing the location and cause of the failure.
	 */
	std::string getError(std::string error);

	/**
	 * Sets the maximum number of times a command will attempts to be sent to the EPOS.
	 *
	 * The EPOS controllers seem to be somewhat finicky in Linux. In some systems, the first few command
	 * attempts will fail. We could just keep them in a continuous while loop, but if there is another problem
	 * with the motors or the controllers, they will be stuck in an infinite loop. For this reason, each of
	 * our commands contains a loop that, if a command fails to send, will retry a specific number of times.
	 * This method sets that number.
	 *
	 * @param commandAttempts The maximum number of times a method will attempt to send a command before failing.
	 */
	void setMaxCommandAttempts(unsigned short commandAttempts);

	/**
	 * Sets the speed of the motor in rpm.
	 *
	 * This method will set the motor to a specific speed, denoted by setpoint. It is important to note that this speed
	 * is in rotations per minute, and is of only the motor. The final output speed will be affected by any gearboxes
	 * attached to it.
	 *
	 * @param setpoint The velocity, in rpm, that the motor is to rotate at.
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool setVelocity(long int setpoint);

	/**
	 * Sets the position of the motor in encoder counts.
	 *
	 * This method will tell the motor to go to a position, specified by the setpoint. The setpoint is in encoder
	 * counts, so it will be up to the user to translate this into angles, distances, etc. The absolute parameter
	 * allows the user to specify whether or not they would like their motor to travel to specific point. If absolute
	 * is true, it will go to the specified setpoint, regardless of the current number of counts on the motor. If it
	 * is false, it will move setpoint number of counts from its current position. Immediately specifies whether or
	 * not the motor should finish any move action it is currently undertaking, or if it should cancel the previous action
	 * and move directly to the setpoint.
	 *
	 * @param setpoint The position, in encoder counts, that the motor is to move to.
	 * @param absolute True if the motor is to move to setpoint counts, false if setpoint counts relative to its current position
	 * @param immediately True if motor is to begin moving to setpoint immediately, false if it is to wait until current action is complete.
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool setPosition(long int setpoint, bool absolute, bool immediately);

	/**
	 * Sets the enable state of the motor.
	 *
	 * This method can be used to enable or disable to motor. If the motor is disabled, it can still send current,
	 * velocity, and position data, but cannot be sent move commands. Also, when the motor is disabled, it does NOT brake,
	 * and can be backdriven.
	 *
	 * @param enabled Boolean indicating desired enable state of motor. True is enabled, False if disabled.
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool setEnableState(bool enabled);

	/**
	 * Sets the Position Profile of the motor.
	 *
	 * This method sets the parameters that the motor uses when the Position Profile is activated. This allows the user
	 * to configure how they want the motors to move when in Position Profile Mode.
	 *
	 * @param positionProfileVelocity The velocity the motor will travel at when in Position Profile Mode
	 * @param positionProfileAcceleration The acceleration of the motor when in Position Profile Mode
	 * @param positionProfileDeceleration The deceleration of the motor when in Position Profile Mode
	 */
	bool setPositionProfile(int positionProfileVelocity,int positionProfileAcceleration,int positionProfileDeceleration);

	/**
	 * Sets the Velocity Profile of the motor.
	 *
	 * This method sets the parameters that the motor uses when the Velocity Profile is activated. This allows the user
	 * to configure how they want the motors to move when in Velocity Profile Mode.
	 *
	 * @param velocityProfileAcceleration The acceleration of the motor when in Velocity Profile Mode
	 * @param velocityProfileDeceleration The deceleration of the motor when in Velocity Profile Mode
	 */
	bool setVelocityProfile(int velocityProfileAcceleration,int velocityProfileDeceleration);

	/**
	 * Sets the velocity the motor will travel at in Position Profile Mode.
	 *
	 * @param positionProfileVelocity The velocity the motor will travel at when in Position Profile Mode
	 */
	bool setPositionProfileVelocity(unsigned long positionProfileVelocity);

	/**
	 * Sets the acceleration the motor use when in Position Profile Mode.
	 *
	 * @param positionProfileAcceleration The acceleration the motor will use when in Position Profile Mode
	 */
	bool setPositionProfileAcceleration(unsigned long positionProfileAcceleration);

	/**
	 * Sets the deceleration the motor use when in Position Profile Mode.
	 *
	 * @param positionProfileDeceleration The deceleration the motor will use when in Position Profile Mode
	 */
	bool setPositionProfileDeceleration(unsigned long positionProfileDeceleration);

	/**
	 * Sets the acceleration the motor use when in Velocity Profile Mode.
	 *
	 * @param velocityProfileAcceleration The acceleration the motor will use when in Velocity Profile Mode
	 */
	bool setVelocityProfileAcceleration(unsigned long velocityProfileAcceleration);

	/**
	 * Sets the deceleration the motor use when in Velocity Profile Mode.
	 *
	 * @param velocityProfileDeceleration The deceleration the motor will use when in Velocity Profile Mode
	 */
	bool setVelocityProfileDeceleration(unsigned long velocityProfileDeceleration);

	/**
	 * Returns the position of the motor in encoder counts.
	 *
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool getPosition(long* position);

	/**
	 * Returns the velocity of the motor in rpm.
	 *
	 * This method returns the velocity of the motor in rpm. It is important to note that this value does
	 * NOT include any gearbox attached to the motor, only the motor itself.
	 *
	 * @param velocity Holder for the velocity of the motor in rpm.
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool getVelocity(long* velocity);

	/**
	 * Returns the current being drawn by the motor in mA.
	 *
	 * @param current
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool getCurrent(short* current);

	/**
	 * Returns the enable state of the motor
	 *
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool getEnableState(bool* enabled);

	/**
	 * Clears all faults on the EPOS.
	 *
	 * Sometimes, the EPOS will go into a fault state if there is a device or function failure. Fortunately,
	 * it is possible to clear these faults. This method clears any faults it can on the EPOS.
	 *
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool clearFaults();

	/**
	 * Gets the fault state of the EPOS
	 *
	 * This method checks the fault state of the EPOS, and returns whether or not the EPOS is ina  fault state.
	 *
	 * @return A boolean indicating the fault state of the EPOS, True if it is in a fault state, False if it is not.
	 */
	bool isInFault();

	/**
	 * Prints the faults on the EPOS
	 *
	 * This method prints any fault or error it finds on the EPOS to ROS_ERROR.
	 *
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool printFaults();
	virtual ~EPOS();
	void dynamicCallback(EposManager::EposManagerConfig &config, uint32_t level);
	/**
	 * Returns a boolean indicating whether or not the EPOS has been initialized
	 *
	 * @return A boolean indicating initialization of EPOS. True if initialized, False if not.
	 */
	bool isEPOSInitialized();
	/**
	 * Initializes the EPOS from parameters stored on the parameter server.
	 *
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool reinitialize();
	/**
	 * Finds the state of the EPOS and stores it in the state variable
	 *
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool getState();
	unsigned short state;



private:
	ros::NodeHandle nh;
	unsigned long errorCode;
	int maxCommandAttempts;
	void* keyHandle;
	bool isInitialized;
	std::string motorName;
	bool isEnabled;
	bool isFaulted;
	char operatingMode;
	int nodeID;


	/************************ MOTOR PARAMETERS ****************************/

	// Select one from the following motor types:
	// DC Motor 						(MT_DC_MOTOR)
	// Brushless w/Sinus Commutation 	(MT_EC_SINUS_COMMUTATED_MOTOR)
	// Brushless w/Block Commutation 	(MT_EC_BLOCK_COMMUTATED_MOTOR)
	int motorType;

	int motorNominalCurrent;
	int motorMaxCurrent;
	int motorThermalTimeConstant;
	int motorNumberPolePairs;

	/************************ SENSOR PARAMETERS ****************************/

	// Select one from the following sensor types:
	// None								(ST_UNKNOWN)
	// Inc. Encoder 3 Channel 			(ST_INC_ENCODER_3CHANNEL)
	// Inc. Encoder 2 Channel 			(ST_INC_ENCODER_2CHANNEL)
	// Hall Effect						(ST_HALL_SENSORS)
	// SSI Abs. Encoder Binary			(ST_SSI_ABS_ENCODER_BINARY)
	// SSI Abs. Encoder Gray			(ST_SSI_ABS_ENCODER_GRAY)
	int sensorType;

	int sensorInversion;
	int sensorIncEncoderResolution;
	int sensorSSIEncoderDataRate;
	int sensorSSIEncoderNumberMultiTurnBits;
	int sensorSSIEncoderNumberSingleTurnBits;

	int maxFollowingError;
	int maxProfileVelocity;
	int maxAcceleration;

	/**************************** REGULATION ******************************/

	// These settings should only be set from the file if they are known,
	// otherwise the regulation tuning utility should be used

	// Position Regulation Settings
	int positionP;
	int positionI;
	int positionD;

	// Velocity Regulation Settings
	int velocityP;
	int velocityI;

	// Current Regulation Settings
	int currentP;
	int currentI;

	int positionProfileVelocity;
	int positionProfileAcceleration;
	int positionProfileDeceleration;
	int velocityProfileAcceleration;
	int velocityProfileDeceleration;

	bool useQuickstop;
	bool quickStopActiveLow;
	int quickstopPin;

	dynamic_reconfigure::Server<EposManager::EposManagerConfig> server;
	dynamic_reconfigure::Server<EposManager::EposManagerConfig>::CallbackType callbackManager;
	EposManager::EposManagerConfig currentConfig;

	/*******************METHODS******************/

	/**
	 * Converts an EPOS error code into the corresponding error string
	 *
	 * @param error The error code on the EPOS
	 * @return errorStr A string explaining the error corresponding to the error code
	 */
	std::string getErrorInfo(unsigned long error){
		std::string errorStr;
		char* errorInfo = new char[400];
		VCS_GetErrorInfo(error,errorInfo,400);
		errorStr.append(errorInfo);
		delete errorInfo;
		return errorStr;
	}

	/**
	 * Sets the sensor type on the motor
	 *
	 * This method sets the sensor type of the motor (hall, incremental, absolute, etc). The
	 * acceptable values can be found in Definitions.h from lines 305-310.
	 *
	 * @param sensorType The sensor type of the motor
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool setSensorType(int sensorType){
		int loopCounter=0;
		while(!VCS_SetSensorType(keyHandle, nodeID, sensorType, &errorCode)){
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Sensor Type: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Sensor_Type", sensorType);
		return true;
	}

	bool setIncEncoderParameters(int sensorIncEncoderResolution, int sensorInversion){
		int loopCounter=0;
		while(!VCS_SetIncEncoderParameter(keyHandle, nodeID, sensorIncEncoderResolution,
				sensorInversion, &errorCode)){
			if (loopCounter++ > maxCommandAttempts){
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Incremental Encoder Parameters: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Encoder_Resolution",sensorIncEncoderResolution);
		nh.setParam("Sensor_Inversion",sensorInversion);
		return true;
	}

	bool setHallSensorParameters(int sensorInversion){
		int loopCounter=0;
		while(!VCS_SetHallSensorParameter(keyHandle, nodeID, sensorInversion, &errorCode)){
			if (loopCounter++ > maxCommandAttempts){
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Hall Sensor Parameters: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Sensor_Inversion",sensorInversion);
		return true;
	}

	bool setSsiAbsEncoderParameters(int sensorSSIEncoderDataRate, int sensorSSIEncoderNumberMultiTurnBits,
			int sensorSSIEncoderNumberSingleTurnBits, int sensorInversion){
		int loopCounter=0;
		while(!VCS_SetSsiAbsEncoderParameter(keyHandle, nodeID, sensorSSIEncoderDataRate,
				sensorSSIEncoderNumberMultiTurnBits, sensorSSIEncoderNumberSingleTurnBits,
				sensorInversion, &errorCode)){
			if (loopCounter++ > maxCommandAttempts){
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " SSI Absolute Encoder Parameters: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Encoder_Number_Multiturn_Bits",sensorSSIEncoderNumberMultiTurnBits);
		nh.setParam("Encoder_Number_Single_Turn_Bits",sensorSSIEncoderNumberSingleTurnBits);
		nh.setParam("Encoder_Data_Rate", sensorSSIEncoderDataRate);
		nh.setParam("Sensor_Inversion", sensorInversion);
		return true;
	}

	bool setOperationMode(char operationMode) {
		int loopCounter = 0;
		while (!VCS_SetOperationMode(keyHandle, nodeID, operationMode,
				&errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Operation Mode: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool setMotorType(int motorType) {
		int loopCounter = 0;
		while (!VCS_SetMotorType(keyHandle, nodeID, motorType, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Motor Type: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Motor_Type",motorType);
		return true;
	}

	bool setDCMotorParameters(int motorNominalCurrent, int motorMaxCurrent, int motorThermalTimeConstant) {
		int loopCounter = 0;
		while (!VCS_SetDcMotorParameter(keyHandle, nodeID, motorNominalCurrent,
				motorMaxCurrent, motorThermalTimeConstant, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " DC Motor Parameters: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Motor_Nominal_Current", motorNominalCurrent);
		nh.setParam("Motor_Max_Current", motorMaxCurrent);
		nh.setParam("Motor_Thermal_Time_Constant", motorThermalTimeConstant);
		return true;
	}

	bool setECMotorParameters(int motorNominalCurrent, int motorMaxCurrent, int motorThermalTimeConstant,
			int motorNumberPolePairs) {
		int loopCounter = 0;
		while (!VCS_SetEcMotorParameter(keyHandle, nodeID, motorNominalCurrent,
				motorMaxCurrent, motorThermalTimeConstant,
				motorNumberPolePairs, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " EC Motor Parameters: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Motor_Nominal_Current", motorNominalCurrent);
		nh.setParam("Motor_Max_Current", motorMaxCurrent);
		nh.setParam("Motor_Thermal_Time_Constant", motorThermalTimeConstant);
		nh.setParam("Motor_Number_Pole_Pairs", motorNumberPolePairs);
		return true;
	}

	bool saveParameters() {
		int loopCounter = 0;
		while (!VCS_Store(keyHandle, nodeID, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not Store Node " << nodeID << " Parameters: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool setMaxVelocity(int maxVelocity) {
		int loopCounter = 0;
		while (!VCS_SetMaxProfileVelocity(keyHandle, nodeID, maxVelocity,
				&errorCode)) {
			ROS_WARN("Command failed to set Maximum Velocity. Retrying...");
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Maximum Velocity: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Max_Profile_Velocity", maxVelocity);
		this->maxProfileVelocity = maxVelocity;
		return true;
	}

	bool setMaxAcceleration(int maxAcceleration) {
		int loopCounter = 0;
		while (!VCS_SetMaxAcceleration(keyHandle, nodeID, maxAcceleration,
				&errorCode)) {
			ROS_WARN("Command failed to set Maximum Acceleration. Retrying...");
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Maximum Acceleration: " << getErrorInfo(errorCode));

				return false;
			}
		}
		nh.setParam("Max_Acceleration", maxAcceleration);
		this->maxAcceleration = maxAcceleration;
		return true;
	}

	/**
	 * Sets the maximum following error of the encoder.
	 * TODO: Finish this comment
	 * This method sets the maximum following error of the encoder. The following error is the difference
	 *
	 */
	bool setMaxFollowingError(int maxFollowingError) {
		int loopCounter = 0;
		while (!VCS_SetMaxFollowingError(keyHandle, nodeID, maxFollowingError,
				&errorCode)) {
			ROS_WARN("Command failed to set Maximum Following Error. Retrying...");
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Maximum Following Error: " << getErrorInfo(errorCode));

				return false;
			}
		}
		nh.setParam("Max_Following_Error", maxFollowingError);
		this->maxFollowingError = maxFollowingError;
		return true;
	}

	bool activateProfileVelocityMode(){
		int loopCounter=0;
		while(!VCS_ActivateProfileVelocityMode(keyHandle, nodeID, &errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_WARN_STREAM("Could not activate Profile Velocity Mode for Node: " <<nodeID<< " : " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool moveWithVelocity(long int setpoint){
		int loopCounter=0;
		while(!VCS_MoveWithVelocity(keyHandle, nodeID, setpoint, &errorCode)){
			ROS_WARN_STREAM("Node " << nodeID << " could not set velocity. Retrying..." );
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not set Node " << nodeID << " Velocity: " << getErrorInfo(errorCode));
				return false;
			}
		}

		return true;
	}

	bool activateProfilePositionMode(){
		int loopCounter=0;
		while(!VCS_ActivateProfilePositionMode(keyHandle, nodeID, &errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_WARN_STREAM("Could not activate Profile Position Mode for Node: " <<nodeID<< " : " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool moveToPosition(long int setpoint, int absolute, int immediately){
		int loopCounter=0;
		while(!VCS_MoveToPosition(keyHandle, nodeID, setpoint,absolute, immediately, &errorCode)){
			ROS_WARN_STREAM("Node " << nodeID << " could not set velocity. Retrying..." );
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not move to Position: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool setVelocityPID(int pGain, int iGain){
		int loopCounter=0;
		while(!VCS_SetVelocityRegulatorGain(keyHandle, nodeID, pGain, iGain, &errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not set Velocity PID: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Velocity_P",velocityP);
		nh.setParam("Velocity_I",velocityI);
		return true;
	}

	bool setPositionPID(int pGain, int iGain, int dGain){
		int loopCounter=0;
		while(!VCS_SetPositionRegulatorGain(keyHandle, nodeID, pGain, iGain, dGain, &errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not set Position PID: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Position_P",positionP);
		nh.setParam("Position_I",positionI);
		nh.setParam("Position_D",positionD);
		return true;
	}

	bool setCurrentPID(int pGain, int iGain){
		int loopCounter=0;
		while(!VCS_SetCurrentRegulatorGain(keyHandle, nodeID, pGain, iGain, &errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not set Current PID: " << getErrorInfo(errorCode));
				return false;
			}
		}
		nh.setParam("Current_P",currentP);
		nh.setParam("Current_I",currentI);
		return true;
	}

	/**
	 * Sets a specified pin to be in QuickStop mode.
	 *
	 * @param quickstopPin The pin to be set
	 * @param activeLow True if quickstop is to be activated when pin is low, false if when pin is high
	 * @param enable Boolean indicating if quickstop mode should be enabled.
	 * @return A boolean indicating whether or not the command succeeded.
	 */
	bool setQuickstop(int quickstopPin, bool activeLow, bool enable){
		int loopCounter=0;
		while(!VCS_DigitalInputConfiguration(keyHandle,nodeID, quickstopPin,DIC_QUICK_STOP,1,int(activeLow),int(enable),&errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not set Quickstop: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadVelocityPIDFromEpos(){
		int loopCounter=0;
		while(!VCS_GetVelocityRegulatorGain(keyHandle, nodeID,(unsigned short*) &velocityP,(unsigned short*) &velocityI, &errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not load Velocity PID from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadPositionPIDFromEpos(){
		int loopCounter=0;
		while(!VCS_GetPositionRegulatorGain(keyHandle, nodeID,(unsigned short*) &positionP,(unsigned short*)&positionI,(unsigned short*)&positionD, &errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not load Position PID from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadCurrentPIDFromEpos(){
		int loopCounter=0;
		while(!VCS_GetCurrentRegulatorGain(keyHandle, nodeID, (unsigned short*)&currentP,(unsigned short*) &currentI,&errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not load Current PID from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadSensorTypeFromEpos(){
		int loopCounter=0;
		while(!VCS_GetSensorType(keyHandle,nodeID, (unsigned short*) &sensorType,&errorCode)){
			if(loopCounter++>maxCommandAttempts){
				ROS_ERROR_STREAM("Could not load Sensor Type from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadIncEncoderParametersFromEpos(){
		int loopCounter=0;
		while(!VCS_GetIncEncoderParameter(keyHandle, nodeID, (unsigned long*) &sensorIncEncoderResolution,
				&sensorInversion, &errorCode)){
			if (loopCounter++ > maxCommandAttempts){
				ROS_ERROR_STREAM("Could not load Incremental Encoder Parameters from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadHallSensorParametersFromEpos(){
		int loopCounter=0;
		while(!VCS_GetHallSensorParameter(keyHandle, nodeID, &sensorInversion, &errorCode)){
			if (loopCounter++ > maxCommandAttempts){
				ROS_ERROR_STREAM("Could not Hall Sensor Parameters from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadSsiAbsEncoderParametersFromEpos(){
		int loopCounter=0;
		while(!VCS_GetSsiAbsEncoderParameter(keyHandle, nodeID, (unsigned short*)&sensorSSIEncoderDataRate,
				(unsigned short*)&sensorSSIEncoderNumberMultiTurnBits,(unsigned short*) &sensorSSIEncoderNumberSingleTurnBits,
				&sensorInversion, &errorCode)){
			if (loopCounter++ > maxCommandAttempts){
				ROS_ERROR_STREAM("Could not load SSI Abs Encoder Parameters from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadMotorTypeFromEpos(){
		int loopCounter = 0;
		while (!VCS_GetMotorType(keyHandle, nodeID, (unsigned short*) &motorType, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not load Motor Type from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadDCMotorParametersFromEpos(){
		int loopCounter = 0;
		while (!VCS_GetDcMotorParameter(keyHandle, nodeID, (unsigned short*) &motorNominalCurrent,
				(unsigned short*) &motorMaxCurrent,(unsigned short*) &motorThermalTimeConstant, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not load DC Motor Parameters from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadECMotorParametersFromEpos(){
		int loopCounter = 0;
		while (!VCS_GetEcMotorParameter(keyHandle, nodeID,(unsigned short*) &motorNominalCurrent,
				(unsigned short*)&motorMaxCurrent,(unsigned short*) &motorThermalTimeConstant,
				(unsigned char*)&motorNumberPolePairs, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not load EC Motor Parameters from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadMaxFollowingErrorFromEpos(){
		int loopCounter = 0;
		while (!VCS_GetMaxFollowingError(keyHandle, nodeID, (unsigned long*) &maxFollowingError,
				&errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not load Maximum Following Error from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadVelocityProfileFromEpos(){
		int loopCounter=0;
		while(!VCS_GetVelocityProfile(keyHandle, nodeID, (unsigned long*) &velocityProfileAcceleration, (unsigned long*) &velocityProfileDeceleration, &errorCode)){
			if (loopCounter++ > maxCommandAttempts){
				ROS_ERROR_STREAM("Could not load Velocity Profile from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadPositionProfileFromEpos(){
		int loopCounter=0;
		while(!VCS_GetPositionProfile(keyHandle, nodeID, (unsigned long*) &positionProfileVelocity, (unsigned long*) &positionProfileAcceleration,(unsigned long*) &positionProfileDeceleration, &errorCode)){
			if (loopCounter++ > maxCommandAttempts){
				ROS_ERROR_STREAM("Could not load Position Profile from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadMaxAccelerationFromEpos(){
		int loopCounter = 0;
		while (!VCS_GetMaxAcceleration(keyHandle, nodeID,(unsigned long*) &maxAcceleration,
				&errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not load Max Acceleration from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadMaxVelocityFromEpos(){
		int loopCounter = 0;
		while (!VCS_GetMaxProfileVelocity(keyHandle, nodeID, (unsigned long*)&maxProfileVelocity, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				ROS_ERROR_STREAM("Could not load Max Velocity from EPOS: " << getErrorInfo(errorCode));
				return false;
			}
		}
		return true;
	}

	bool loadParametersFromEpos(){
		if(keyHandle==0){
			ROS_ERROR_STREAM("Keyhandle Not Initialized. Cannot Retrieve Parameters from EPOS."<<
					"Please Initialize Keyhandle");
			return false;
		}

		ROS_INFO_STREAM("Loading Parameters from EPOS "<<nodeID);

		loadMotorTypeFromEpos();
		loadDCMotorParametersFromEpos();
		loadECMotorParametersFromEpos();

		loadSensorTypeFromEpos();
		loadSsiAbsEncoderParametersFromEpos();
		loadHallSensorParametersFromEpos();
		loadIncEncoderParametersFromEpos();

		loadPositionPIDFromEpos();
		loadVelocityPIDFromEpos();
		loadCurrentPIDFromEpos();

		loadMaxAccelerationFromEpos();
		loadMaxFollowingErrorFromEpos();
		loadMaxVelocityFromEpos();

		loadPositionProfileFromEpos();
		loadVelocityProfileFromEpos();

		return true;
	}


};

#endif /* EPOS_H_ */
