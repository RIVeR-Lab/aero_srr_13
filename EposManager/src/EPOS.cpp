/*
 * EPOS.cpp
 *
 *  Created on: Jan 20, 2012
 *      Author: oryx
 */

#include "EposManager/EPOS.h"
#include <errno.h>
#include <signal.h>
using namespace std;

void EPOS::dynamicCallback(EposManager::EposManagerConfig &config, uint32_t level){
	if(isInitialized){
		if(currentConfig.Position_Profile_Velocity!=config.Position_Profile_Velocity){
			setPositionProfileVelocity(config.Position_Profile_Velocity);
		}
		if(currentConfig.Position_Profile_Deceleration!=config.Position_Profile_Deceleration){
			setPositionProfileDeceleration(config.Position_Profile_Deceleration);
		}
		if(currentConfig.Position_Profile_Acceleration!=config.Position_Profile_Acceleration){
			setPositionProfileAcceleration(config.Position_Profile_Acceleration);
		}
		if(currentConfig.Velocity_Profile_Acceleration!=config.Velocity_Profile_Acceleration){
			setVelocityProfileAcceleration(config.Velocity_Profile_Acceleration);
		}
		if(currentConfig.Velocity_Profile_Deceleration!=config.Velocity_Profile_Deceleration){
			setVelocityProfileDeceleration(config.Velocity_Profile_Deceleration);
		}
		if(currentConfig.Disable_Motor!=config.Disable_Motor){
			clearFaults();
			setEnableState(!config.Disable_Motor);
		}
		if(currentConfig.Reinitialize_Motor != config.Reinitialize_Motor
				&& config.Reinitialize_Motor == true){
			if(reinitialize()) setEnableState(true);
		}

	}
	currentConfig = config;
}

EPOS::EPOS(ros::NodeHandle _nh):server(_nh) {
	maxCommandAttempts = 5;
	errorCode = 0;
	nh = _nh;
	isInitialized=false;
	callbackManager = boost::bind(&EPOS::dynamicCallback, this, _1 ,_2);
	server.setCallback(callbackManager);
}


bool EPOS::reinitialize() {
	if(!clearFaults())
		return false;
	if (nh.hasParam("Motor_Name"))
		nh.getParam("Motor_Name", motorName);
	if(!setEnableState(false))
		return false;
	//////MOTOR PARAMETERS/////////
	if (nh.hasParam("Motor_Type")){
		nh.getParam("Motor_Type", motorType);
		if(!setMotorType(motorType)) return false;
	}
	else {
		loadMotorTypeFromEpos();
		if(!setMotorType(motorType))return false;
	}
	switch(motorType){
		case MT_DC_MOTOR:
			if(nh.hasParam("Motor_Nominal_Current") && nh.hasParam("Motor_Max_Current") && nh.hasParam("Motor_Thermal_Time_Constant")){
				nh.getParam("Motor_Nominal_Current", motorNominalCurrent);
				nh.getParam("Motor_Max_Current", motorMaxCurrent);
				nh.getParam("Motor_Thermal_Time_Constant", motorThermalTimeConstant);
				if(!setDCMotorParameters(motorNominalCurrent,motorMaxCurrent,motorThermalTimeConstant))return false;
			}
			else if (nh.hasParam("Motor_Nominal_Current") || nh.hasParam("Motor_Max_Current") || nh.hasParam("Motor_Thermal_Time_Constant")){
				if (nh.hasParam("Motor_Nominal_Current"))
					nh.getParam("Motor_Nominal_Current", motorNominalCurrent);
				if (nh.hasParam("Motor_Max_Current"))
					nh.getParam("Motor_Max_Current", motorMaxCurrent);
				if (nh.hasParam("Motor_Thermal_Time_Constant"))
					nh.getParam("Motor_Thermal_Time_Constant", motorThermalTimeConstant);
				if(!setDCMotorParameters(motorNominalCurrent,motorMaxCurrent,motorThermalTimeConstant))return false;
			}
			break;
		case MT_EC_BLOCK_COMMUTATED_MOTOR:
			if(nh.hasParam("Motor_Nominal_Current") && nh.hasParam("Motor_Max_Current") && nh.hasParam("Motor_Thermal_Time_Constant")&& nh.hasParam("Motor_Number_Pole_Pairs")){
				nh.getParam("Motor_Nominal_Current", motorNominalCurrent);
				nh.getParam("Motor_Max_Current", motorMaxCurrent);
				nh.getParam("Motor_Thermal_Time_Constant", motorThermalTimeConstant);
				nh.getParam("Motor_Number_Pole_Pairs", motorNumberPolePairs);
				if(!setECMotorParameters(motorNominalCurrent,motorMaxCurrent,
					motorThermalTimeConstant,motorNumberPolePairs)) return false;
			}
			else if(nh.hasParam("Motor_Nominal_Current") || nh.hasParam("Motor_Max_Current") || nh.hasParam("Motor_Thermal_Time_Constant") || nh.hasParam("Motor_Number_Pole_Pairs")){
				if (nh.hasParam("Motor_Nominal_Current"))
					nh.getParam("Motor_Nominal_Current", motorNominalCurrent);
				if (nh.hasParam("Motor_Max_Current"))
					nh.getParam("Motor_Max_Current", motorMaxCurrent);
				if (nh.hasParam("Motor_Thermal_Time_Constant"))
					nh.getParam("Motor_Thermal_Time_Constant", motorThermalTimeConstant);
				if (nh.hasParam("Motor_Number_Pole_Pairs"))
					nh.getParam("Motor_Number_Pole_Pairs", motorNumberPolePairs);
				if(!setECMotorParameters(motorNominalCurrent,motorMaxCurrent,
					motorThermalTimeConstant,motorNumberPolePairs)) return false;
			}
			break;
		case MT_EC_SINUS_COMMUTATED_MOTOR:
			if(nh.hasParam("Motor_Nominal_Current") && nh.hasParam("Motor_Max_Current") && nh.hasParam("Motor_Thermal_Time_Constant")&& nh.hasParam("Motor_Number_Pole_Pairs")){
				nh.getParam("Motor_Nominal_Current", motorNominalCurrent);
				nh.getParam("Motor_Max_Current", motorMaxCurrent);
				nh.getParam("Motor_Thermal_Time_Constant", motorThermalTimeConstant);
				nh.getParam("Motor_Number_Pole_Pairs", motorNumberPolePairs);
				if(!setECMotorParameters(motorNominalCurrent,motorMaxCurrent,
					motorThermalTimeConstant,motorNumberPolePairs)) return false;
			}
			else if(nh.hasParam("Motor_Nominal_Current") || nh.hasParam("Motor_Max_Current") || nh.hasParam("Motor_Thermal_Time_Constant") || nh.hasParam("Motor_Number_Pole_Pairs")){
				if (nh.hasParam("Motor_Nominal_Current"))
					nh.getParam("Motor_Nominal_Current", motorNominalCurrent);
				if (nh.hasParam("Motor_Max_Current"))
					nh.getParam("Motor_Max_Current", motorMaxCurrent);
				if (nh.hasParam("Motor_Thermal_Time_Constant"))
					nh.getParam("Motor_Thermal_Time_Constant", motorThermalTimeConstant);
				if (nh.hasParam("Motor_Number_Pole_Pairs"))
					nh.getParam("Motor_Number_Pole_Pairs", motorNumberPolePairs);
				if(!setECMotorParameters(motorNominalCurrent,motorMaxCurrent,
					motorThermalTimeConstant,motorNumberPolePairs)) return false;
			}
			break;
		default: ROS_ERROR_STREAM("No Valid Motor Parameters Available");
	}



	///////SENSOR PARAMETERS/////////////
	if(nh.hasParam("Sensor_Type")){
		nh.getParam("Sensor_Type", sensorType);
		if(!setSensorType(sensorType))return false;
	}
	else{
		loadSensorTypeFromEpos();
		if(!setSensorType(sensorType))return false;
	}

	switch(sensorType){
	/////3 CHANNEL ENCODER PARAMETERS////////////
	case ST_INC_ENCODER_3CHANNEL:
		if(nh.hasParam("Encoder_Resolution") && nh.hasParam("Sensor_Inversion")){
			nh.getParam("Encoder_Resolution", sensorIncEncoderResolution);
			nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setIncEncoderParameters(sensorIncEncoderResolution,sensorInversion))return false;
		}
		else if (nh.hasParam("Encoder_Resolution") || nh.hasParam("Sensor_Inversion")){
			loadIncEncoderParametersFromEpos();
			if (nh.hasParam("Encoder_Resolution"))
				nh.getParam("Encoder_Resolution", sensorIncEncoderResolution);
			if (nh.hasParam("Sensor_Inversion"))
				nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setIncEncoderParameters(sensorIncEncoderResolution,sensorInversion))return false;
		}
		break;
	/////2 CHANNEL ENCODER PARAMETERS////////////
	case ST_INC_ENCODER_2CHANNEL:
		if(nh.hasParam("Encoder_Resolution") && nh.hasParam("Sensor_Inversion")){
			nh.getParam("Encoder_Resolution", sensorIncEncoderResolution);
			nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setIncEncoderParameters(sensorIncEncoderResolution,sensorInversion))return false;
		}
		else if (nh.hasParam("Encoder_Resolution") || nh.hasParam("Sensor_Inversion")){
			loadIncEncoderParametersFromEpos();
			if (nh.hasParam("Encoder_Resolution"))
				nh.getParam("Encoder_Resolution", sensorIncEncoderResolution);
			if (nh.hasParam("Sensor_Inversion"))
				nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setIncEncoderParameters(sensorIncEncoderResolution,sensorInversion))return false;
		}
		break;
	/////HALL SENSOR PARAMETERS////////////
	case ST_HALL_SENSORS:
		if(nh.hasParam("Sensor_Inversion")){
			nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setHallSensorParameters(sensorInversion))return false;
		}
		break;
	case ST_SSI_ABS_ENCODER_BINARY:
		if(nh.hasParam("Encoder_Number_Multiturn_Bits")&& nh.hasParam("Encoder_Number_Single_Turn_Bits")
				&& nh.hasParam("Encoder_Data_Rate") && nh.hasParam("Sensor_Inversion")){
			nh.getParam("Encoder_Data_Rate", sensorSSIEncoderDataRate);
			nh.getParam("Encoder_Number_Multiturn_Bits",sensorSSIEncoderNumberMultiTurnBits);
			nh.getParam("Encoder_Number_Single_Turn_Bits",sensorSSIEncoderNumberSingleTurnBits);
			nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setSsiAbsEncoderParameters(sensorSSIEncoderDataRate,sensorSSIEncoderNumberMultiTurnBits,
					sensorSSIEncoderNumberSingleTurnBits,sensorInversion))return false;
		}
		else if (nh.hasParam("Encoder_Number_Multiturn_Bits")|| nh.hasParam("Encoder_Number_Single_Turn_Bits")
				|| nh.hasParam("Encoder_Data_Rate")|| nh.hasParam("Sensor_Inversion")){
			loadSsiAbsEncoderParametersFromEpos();
			if (nh.hasParam("Encoder_Data_Rate"))
				nh.getParam("Encoder_Data_Rate", sensorSSIEncoderDataRate);
			if (nh.hasParam("Encoder_Number_Multiturn_Bits"))
				nh.getParam("Encoder_Number_Multiturn_Bits",sensorSSIEncoderNumberMultiTurnBits);
			if (nh.hasParam("Encoder_Number_Single_Turn_Bits"))
				nh.getParam("Encoder_Number_Single_Turn_Bits",sensorSSIEncoderNumberSingleTurnBits);
			if (nh.hasParam("Sensor_Inversion"))
				nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setSsiAbsEncoderParameters(sensorSSIEncoderDataRate,sensorSSIEncoderNumberMultiTurnBits,
					sensorSSIEncoderNumberSingleTurnBits,sensorInversion))return false;
		}
		break;
	case ST_SSI_ABS_ENCODER_GREY:
		if(nh.hasParam("Encoder_Number_Multiturn_Bits")&& nh.hasParam("Encoder_Number_Single_Turn_Bits")
				&& nh.hasParam("Encoder_Data_Rate") && nh.hasParam("Sensor_Inversion")){
			nh.getParam("Encoder_Data_Rate", sensorSSIEncoderDataRate);
			nh.getParam("Encoder_Number_Multiturn_Bits",sensorSSIEncoderNumberMultiTurnBits);
			nh.getParam("Encoder_Number_Single_Turn_Bits",sensorSSIEncoderNumberSingleTurnBits);
			nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setSsiAbsEncoderParameters(sensorSSIEncoderDataRate,sensorSSIEncoderNumberMultiTurnBits,
					sensorSSIEncoderNumberSingleTurnBits,sensorInversion))return false;
		}
		else if (nh.hasParam("Encoder_Number_Multiturn_Bits")|| nh.hasParam("Encoder_Number_Single_Turn_Bits")
				|| nh.hasParam("Encoder_Data_Rate")|| nh.hasParam("Sensor_Inversion")){
			loadSsiAbsEncoderParametersFromEpos();
			if (nh.hasParam("Encoder_Data_Rate"))
				nh.getParam("Encoder_Data_Rate", sensorSSIEncoderDataRate);
			if (nh.hasParam("Encoder_Number_Multiturn_Bits"))
				nh.getParam("Encoder_Number_Multiturn_Bits",sensorSSIEncoderNumberMultiTurnBits);
			if (nh.hasParam("Encoder_Number_Single_Turn_Bits"))
				nh.getParam("Encoder_Number_Single_Turn_Bits",sensorSSIEncoderNumberSingleTurnBits);
			if (nh.hasParam("Sensor_Inversion"))
				nh.getParam("Sensor_Inversion", sensorInversion);
			if(!setSsiAbsEncoderParameters(sensorSSIEncoderDataRate,sensorSSIEncoderNumberMultiTurnBits,
					sensorSSIEncoderNumberSingleTurnBits,sensorInversion))return false;
		}
		break;
	default:
		//getError("No Acceptable Sensor Parameters Available");
		return false;
	}


	////////SAFETY PARAMETERS//////////////////////
	if (nh.hasParam("Max_Following_Error")){
		nh.getParam("Max_Following_Error", maxFollowingError);
		if(!setMaxFollowingError(maxFollowingError))return false;
	}
	else{
		loadMaxFollowingErrorFromEpos();
		nh.setParam("Max_Following_Error",maxFollowingError);
	}

	if (nh.hasParam("Max_Profile_Velocity")){
		nh.getParam("Max_Profile_Velocity", maxProfileVelocity);
		if(!setMaxVelocity(maxProfileVelocity))return false;
	}
	else{
		loadMaxVelocityFromEpos();
		nh.setParam("Max_Profile_Velocity", maxProfileVelocity);
	}

	if (nh.hasParam("Max_Acceleration")){
		nh.getParam("Max_Acceleration", maxAcceleration);
		if(!setMaxAcceleration(maxAcceleration)) return false;
	}
	else{
		loadMaxAccelerationFromEpos();
		nh.setParam("Max_Acceleration",maxAcceleration);
	}

	////////POSITION PROFILE///////////////////
	loadPositionProfileFromEpos();
	if (nh.hasParam("Position_Profile_Velocity"))
		nh.getParam("Position_Profile_Velocity", positionProfileVelocity);
	if (nh.hasParam("Position_Profile_Acceleration"))
		nh.getParam("Position_Profile_Acceleration",positionProfileAcceleration);
	if (nh.hasParam("Position_Profile_Deceleration"))
		nh.getParam("Position_Profile_Deceleration",positionProfileDeceleration);
	if(!setPositionProfile(positionProfileVelocity,positionProfileAcceleration,positionProfileDeceleration))return false;




	////////////VELOCITY PROFILE///////////////////////
	loadVelocityProfileFromEpos();
	if (nh.hasParam("Velocity_Profile_Acceleration"))
		nh.getParam("Velocity_Profile_Acceleration",velocityProfileAcceleration);
	if (nh.hasParam("Velocity_Profile_Deceleration"))
		nh.getParam("Velocity_Profile_Deceleration",velocityProfileDeceleration);
	if(!setVelocityProfile(velocityProfileAcceleration,velocityProfileDeceleration))return false;


	///////////POSITION PID///////////////////////////
	if (nh.hasParam("Position_P") && nh.hasParam("Position_I") && nh.hasParam("Position_D")){
		nh.getParam("Position_P", positionP);
		nh.getParam("Position_I", positionI);
		nh.getParam("Position_D", positionD);
		if(!setPositionPID(positionP,positionI,positionD))return false;
	}
	else if (nh.hasParam("Position_P") || nh.hasParam("Position_I") || nh.hasParam("Position_D")){
		loadPositionPIDFromEpos();
		if(nh.hasParam("Position_P"))nh.getParam("Position_P",positionP);
		if(nh.hasParam("Position_I"))nh.getParam("Position_I",positionI);
		if(nh.hasParam("Position_D"))nh.getParam("Position_D",positionD);
		if(!setPositionPID(positionP,positionI,positionD))return false;
	}

	////////////VELOCITY PID/////////////////
	if (nh.hasParam("Velocity_P") && nh.hasParam("Velocity_I")){
		nh.getParam("Velocity_P", velocityP);
		nh.getParam("Velocity_I", velocityI);
		if(!setVelocityPID(velocityP,velocityI))return false;
	}
	else if (nh.hasParam("Velocity_P") || nh.hasParam("Velocity_I")){
		loadVelocityPIDFromEpos();
		if(nh.hasParam("Velocity_P"))nh.getParam("Velocity_P",velocityP);
		if(nh.hasParam("Velocity_I"))nh.getParam("Velocity_I",velocityI);
		if(!setVelocityPID(velocityP,velocityI))return false;
	}

	/////////////CURRENT PID/////////////
	if (nh.hasParam("Current_P") && nh.hasParam("Current_I")){
		nh.getParam("Current_P", currentP);
		nh.getParam("Current_I", currentI);
		if(!setCurrentPID(currentP,currentI))return false;
	}
	else if (nh.hasParam("Current_P") || nh.hasParam("Current_I")){
		loadCurrentPIDFromEpos();
		if(nh.hasParam("Current_P"))nh.getParam("Current_P",currentP);
		if(nh.hasParam("Current_I"))nh.getParam("Current_I",currentI);
		if(!setCurrentPID(currentP,currentI))return false;
	}

	if(nh.hasParam("Use_QuickStop")){
		nh.getParam("Use_QuickStop",useQuickstop);
		nh.getParam("Active_Low", quickStopActiveLow);
		if(useQuickstop){
			if(nh.hasParam("QuickStop_Pin")){
				nh.getParam("QuickStop_Pin",quickstopPin);
				setQuickstop(quickstopPin,quickStopActiveLow,true);
			}
			else{
				if(motorName.size())
					ROS_WARN_STREAM("No quickstop pin given for " << motorName << ". Cannot enable quickstop.");
				else
				ROS_WARN_STREAM("No quickstop pin given for motor" << nodeID << ". Cannot enable quickstop.");
			}
		}
		else{
			if(nh.hasParam("QuickStop_Pin")){
				nh.getParam("QuickStop_Pin",quickstopPin);
				setQuickstop(quickstopPin,true,false);
			}
		}
	}

	if(!saveParameters())
		return false;

	isInitialized = true;
	if(motorName.size())
		ROS_INFO_STREAM(motorName << " Successfully Initialized");
	else
		ROS_INFO("Motor %d %s", nodeID, " Successfully Initialized");
	return true;
}


bool EPOS::initializeMotor(void* keyHandle){
	nodeID=-1;
	if(keyHandle==0){
		ROS_ERROR("Keyhandle Not Initialized. Please Inititialize Keyhandle");
		return false;
	}
	this->keyHandle = keyHandle;

	if (nh.hasParam("Node_ID")) nh.getParam("Node_ID", nodeID);
	else {
		ROS_ERROR("Must specify Node ID");
		return false;
	}
	if(!reinitialize())return false;
	if(!setEnableState(true))
		return false;
	return true;

}


int EPOS::getNodeID(){
	return nodeID;
}

string EPOS::getMotorName(){
	return motorName;
}


void EPOS::setMaxCommandAttempts(unsigned short commandAttempts){
	maxCommandAttempts = commandAttempts;
}

bool EPOS::setVelocity(long int setpoint){
	if(!isInitialized){
		if(motorName.size())
			ROS_WARN_STREAM(motorName <<  " is not Initialized. Cannot set velocity");
		else
			ROS_WARN_STREAM("Motor " << nodeID << " is not Initialized. Cannot set velocity");
		return false;
	}
	getState();
	switch(state){
		case ST_DISABLED:
			if(motorName.size())
				ROS_WARN_STREAM(motorName <<  " is not Enabled. Cannot set velocity");
			else
				ROS_WARN_STREAM("Motor " << nodeID << " is not Enabled. Cannot set velocity");
			return false;
		case ST_QUICKSTOP:
			if(motorName.size())
				ROS_WARN_STREAM(motorName <<  " is in QuickStop. Cannot set velocity");
			else
				ROS_WARN_STREAM("Motor " << nodeID << " is in QuickStop. Cannot set velocity");
			return false;
		case ST_FAULT:
			if(motorName.size())
				ROS_WARN_STREAM(motorName <<  " is in Fault State. Cannot set velocity");
			else
				ROS_WARN_STREAM("Motor " << nodeID << " is in Fault State. Cannot set velocity");
			printFaults();
			return false;
		case ST_ENABLED:
			if(setpoint > maxProfileVelocity){
				ROS_WARN("Desired velocity higher than maximum, reducing to %d",maxProfileVelocity);
				setpoint=maxProfileVelocity;
			}
			if(!activateProfileVelocityMode())return false;
			if(!moveWithVelocity(setpoint))return false;
			return true;
		default:
			return false;
	}


}


bool EPOS::setPosition(long int setpoint, bool absolute, bool immediately){
	if(!isInitialized){
		if(motorName.size())
			ROS_WARN_STREAM(motorName <<  " is not Initialized. Cannot set position");
		else
			ROS_WARN_STREAM("Motor " << nodeID << " is not Initialized. Cannot set position");
		return false;
	}
	getState();
	switch(state){
		case ST_DISABLED:
			if(motorName.size())
				ROS_WARN_STREAM(motorName <<  " is not Enabled. Cannot set position");
			else
				ROS_WARN_STREAM("Motor " << nodeID << " is not Enabled. Cannot set position");
			return false;
		case ST_QUICKSTOP:
			if(motorName.size())
				ROS_WARN_STREAM(motorName <<  " is in QuickStop. Cannot set position");
			else
				ROS_WARN_STREAM("Motor " << nodeID << " is in QuickStop. Cannot set position");
			return false;
		case ST_FAULT:
			if(motorName.size())
				ROS_WARN_STREAM(motorName <<  " is in Fault State. Cannot set position");
			else
				ROS_WARN_STREAM("Motor " << nodeID << " is in Fault State. Cannot set position");
			printFaults();
			return false;
		case ST_ENABLED:
			if(!activateProfilePositionMode())return false;
			if(!moveToPosition(setpoint,(int)absolute,(int)immediately))return false;
			return true;
		default:
			return false;
	}
}

bool EPOS::setEnableState(bool enabled){
	int loopCounter=0;
	if (enabled){
		while (!VCS_SetEnableState(keyHandle, nodeID, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts) {
				if(motorName.size())
					ROS_ERROR_STREAM("Could not enable " << motorName);
				else
					ROS_ERROR_STREAM("Could not enable motor " << nodeID <<": " << getErrorInfo(errorCode));
				return false;
			}
		}
	}
	else {
		while (!VCS_SetDisableState(keyHandle, nodeID, &errorCode)) {
			if (loopCounter++ > maxCommandAttempts){
				if(motorName.size())
					ROS_ERROR_STREAM("Could not disable " << motorName << ": " <<getErrorInfo(errorCode));
				else
					ROS_ERROR_STREAM("Could not disable motor " << nodeID <<": " << getErrorInfo(errorCode));
				return false;
			}
		}
	}
	isEnabled = enabled;
	return true;
}

bool EPOS::setPositionProfile(int positionProfileVelocity,int positionProfileAcceleration,int positionProfileDeceleration){
	if(positionProfileVelocity>maxProfileVelocity) {
		ROS_WARN_STREAM("Desired Position Profile Velocity is greater than Maximum Velocity. " <<
				"Setting desired Position Profile Velocity to " << maxProfileVelocity);
		positionProfileVelocity=maxProfileVelocity;
	}

	if(positionProfileAcceleration>maxAcceleration){
		ROS_WARN_STREAM("Desired Position Profile Acceleration is greater than Maximum Acceleration. " <<
				"Setting desired Position Profile Acceleration to " << maxAcceleration);
		positionProfileAcceleration=maxAcceleration;
	}

	if(positionProfileDeceleration>maxAcceleration){
		ROS_WARN_STREAM("Desired Position Profile Deceleration is greater than Maximum Deceleration. " <<
				"Setting desired Position Profile Deceleration to " << maxAcceleration);
		positionProfileDeceleration=maxAcceleration;
	}

	if(!setEnableState(false))return false;

	int loopCounter=0;
	while(!VCS_SetPositionProfile(keyHandle, nodeID, positionProfileVelocity, positionProfileAcceleration, positionProfileDeceleration, &errorCode)){
		if (loopCounter++ > maxCommandAttempts){
			if(motorName.size())
				ROS_ERROR_STREAM("Could not set " << motorName << " Position Profile: " << getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not set motor " << nodeID <<" Position Profile: " << getErrorInfo(errorCode));
			return false;
		}
	}

	if(!saveParameters())return false;

	if(!setEnableState(true))return false;

	nh.setParam("Position_Profile_Velocity",positionProfileVelocity);
	this->positionProfileVelocity=positionProfileVelocity;
	nh.setParam("Position_Profile_Acceleration",positionProfileAcceleration);
	this->positionProfileAcceleration=positionProfileAcceleration;
	nh.setParam("Position_Profile_Deceleration",positionProfileDeceleration);
	this->positionProfileDeceleration=positionProfileDeceleration;

	return true;
}

bool EPOS::setVelocityProfile(int velocityProfileAcceleration,int velocityProfileDeceleration){
	if(velocityProfileAcceleration>maxAcceleration){
		ROS_WARN_STREAM("Desired Velocity Profile Acceleration is greater than Maximum Acceleration. " <<
			"Setting desired Velocity Profile Acceleration to " << maxAcceleration);
		velocityProfileAcceleration=maxAcceleration;
	}
	if(velocityProfileDeceleration>maxAcceleration){
		ROS_WARN_STREAM("Desired Velocity Profile Deceleration is greater than Maximum Deceleration. " <<
			"Setting desired Velocity Profile Deceleration to " << maxAcceleration);
		velocityProfileDeceleration=maxAcceleration;
	}

	if(!setEnableState(false))return false;
	int loopCounter=0;
	while(!VCS_SetVelocityProfile(keyHandle, nodeID, velocityProfileAcceleration, velocityProfileDeceleration, &errorCode)){
		if (loopCounter++ > maxCommandAttempts){
			if(motorName.size())
				ROS_ERROR_STREAM("Could not set " << motorName << " Velocity Profile: " << getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not set motor " << nodeID <<" Velocity Profile: " << getErrorInfo(errorCode));
			return false;
		}
	}

	if(!saveParameters())return false;

	if(!setEnableState(true))return false;

	nh.setParam("Velocity_Profile_Acceleration",velocityProfileAcceleration);
	this->velocityProfileAcceleration=velocityProfileAcceleration;
	nh.setParam("Velocity_Profile_Deceleration",velocityProfileDeceleration);
	this->velocityProfileDeceleration=velocityProfileDeceleration;

	return true;
}

bool EPOS::setPositionProfileVelocity(unsigned long positionProfileVelocity){
	if(setPositionProfile(positionProfileVelocity,positionProfileAcceleration,positionProfileDeceleration))return true;
	else return false;
}

bool EPOS::setPositionProfileAcceleration(unsigned long positionProfileAcceleration){
	if(setPositionProfile(positionProfileVelocity,positionProfileAcceleration,positionProfileDeceleration))return true;
	else return false;
}
bool EPOS::setPositionProfileDeceleration(unsigned long positionProfileDeceleration){
	if(setPositionProfile(positionProfileVelocity,positionProfileAcceleration,positionProfileDeceleration)){
		return true;
	}
	else return false;
}

bool EPOS::setVelocityProfileAcceleration(unsigned long velocityProfileAcceleration){
	if(setVelocityProfile(velocityProfileAcceleration,velocityProfileDeceleration))return true;
	else return false;
}

bool EPOS::setVelocityProfileDeceleration(unsigned long velocityProfileDeceleration){
	if(setVelocityProfile(velocityProfileAcceleration,velocityProfileDeceleration))return true;
	else return false;
}

bool EPOS::getPosition(long* position){
	int loopCounter=0;
	while (!VCS_GetPositionIs(keyHandle, nodeID, position, &errorCode)) {
		if(motorName.size())
			ROS_WARN_STREAM("Could not grab " << motorName << " Position. Retrying...");
		else
			ROS_WARN_STREAM("Could not grab motor " << nodeID <<" Position. Retrying... ");
		if (loopCounter++ > maxCommandAttempts) {
			if(motorName.size())
				ROS_ERROR_STREAM("Could not get " << motorName << " Position: " << getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not get motor " << nodeID <<" Position: " << getErrorInfo(errorCode));
			return false;
		}
	}
	return true;
}

bool EPOS::getVelocity(long* velocity){
	int loopCounter=0;
	while (!VCS_GetVelocityIs(keyHandle, nodeID, velocity, &errorCode)) {
		if(motorName.size())
			ROS_WARN_STREAM("Could not grab " << motorName << " Velocity. Retrying...");
		else
			ROS_WARN_STREAM("Could not grab motor " << nodeID <<" Velocity. Retrying...");
		if (loopCounter++ > maxCommandAttempts){
			if(motorName.size())
				ROS_ERROR_STREAM("Could not get " << motorName << " Velocity: " << getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not get motor " << nodeID <<" Velocity: " << getErrorInfo(errorCode));
			return false;
		}
	}
	return true;
}

bool EPOS::getCurrent(short* current){
	int loopCounter=0;
	while (!VCS_GetCurrentIs(keyHandle, nodeID, current, &errorCode)) {
		if(motorName.size())
			ROS_WARN_STREAM("Could not grab " << motorName << " Current. Retrying...");
		else
			ROS_WARN_STREAM("Could not grab motor " << nodeID <<" Current. Retrying...");

		if (loopCounter++ > maxCommandAttempts) {
			if(motorName.size())
				ROS_ERROR_STREAM("Could not get " << motorName << " Current: " << getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not get motor " << nodeID <<" Current: " << getErrorInfo(errorCode));
			return false;
		}
	}
	return true;
}

bool EPOS::getEnableState(bool* enabled){
	int isEnabled;
	int loopCounter=0;
	while (!VCS_GetEnableState(keyHandle, nodeID, &isEnabled, &errorCode)) {
		if(motorName.size())
			ROS_WARN_STREAM("Could not grab " << motorName << " Enable State. Retrying...");
		else
			ROS_WARN_STREAM("Could not grab motor " << nodeID <<" Enable State. Retrying...");

		if (loopCounter++ > maxCommandAttempts) {
			if(motorName.size())
				ROS_ERROR_STREAM("Could not get " << motorName << " Enable State: " << getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not get motor " << nodeID <<" Enable State: " << getErrorInfo(errorCode));
			return false;
		}
	}
	*enabled = (isEnabled == ST_ENABLED) ? true : false;
	return true;
}

bool EPOS::clearFaults(){
	int loopCounter=0;
	while(!VCS_ClearFault(keyHandle, nodeID, &errorCode)){
		if (loopCounter++ > maxCommandAttempts){
			if(motorName.size())
				ROS_ERROR_STREAM("Could not Clear " << motorName << " Faults: "<< getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not Clear motor " << nodeID <<" Faults: "<< getErrorInfo(errorCode));
			return false;
		}
	}
	return true;
}

bool EPOS::isInFault(){
	int loopCounter=0;
	int isInFault;

	while (!VCS_GetFaultState(keyHandle, nodeID, &isInFault,&errorCode)) {
		if (loopCounter++ > maxCommandAttempts){
			if(motorName.size())
				ROS_ERROR_STREAM("Could not get " << motorName << " Faults State: "<< getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not get motor " << nodeID <<" Fault State : "<< getErrorInfo(errorCode));
			isFaulted=true;
			return true;
		}
	}
	if(isInFault==1){
		isFaulted=true;
		return true;
	}
	else {
		isFaulted=false;
		return false;
	}
}

bool EPOS::printFaults(){
	int loopCounter=0;
	unsigned char numErrors;
	unsigned long functionErrorCode = 0;
	unsigned long deviceErrorCode = 0;

	while (!VCS_GetNbOfDeviceError(keyHandle, nodeID, &numErrors,&errorCode)) {
		if (loopCounter++ > maxCommandAttempts){
			if(motorName.size())
				ROS_ERROR_STREAM("Could not get number of " << motorName << " Faults: "<< getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not get number of motor " << nodeID <<" Faults : "<< getErrorInfo(errorCode));
			return false;
		}
	}
	loopCounter=0;

	for(unsigned char errorNumber = 1; errorNumber <= numErrors; errorNumber++){
		while (!VCS_GetDeviceErrorCode(keyHandle, nodeID, errorNumber, &deviceErrorCode, &functionErrorCode)) {
			if (loopCounter++ > maxCommandAttempts){
				if(motorName.size())
					ROS_ERROR_STREAM("Could not get " << motorName << " Fault Codes: "<< getErrorInfo(errorCode));
				else
					ROS_ERROR_STREAM("Could not get  motor " << nodeID <<" Fault Codes: "<< getErrorInfo(errorCode));
				return false;
			}
		}
		if(deviceErrorCode >0){
			if(motorName.size())
				ROS_ERROR_STREAM(motorName << " in fault state. Device Error: " << getErrorInfo(deviceErrorCode).c_str());
			else
				ROS_ERROR_STREAM("Motor " << nodeID << " in fault state. Device Error: " << getErrorInfo(deviceErrorCode).c_str());
		}
		if(functionErrorCode >0){
			if(motorName.size())
				ROS_ERROR_STREAM(motorName << " in fault state. Function Error: " << getErrorInfo(functionErrorCode).c_str());
			else
				ROS_ERROR_STREAM("Motor " << nodeID << " in fault state. Function Error: " << getErrorInfo(functionErrorCode).c_str());
		}
	}
	return true;
}

bool EPOS::isEPOSInitialized(){
	return isInitialized;
}

bool EPOS::getState(){
	int loopCounter=0;
	while(!VCS_GetState(keyHandle,nodeID,&state,&errorCode)){
		if (loopCounter++ > maxCommandAttempts){
			if(motorName.size())
				ROS_ERROR_STREAM("Could not get " << motorName << " State: "<< getErrorInfo(errorCode));
			else
				ROS_ERROR_STREAM("Could not get motor " << nodeID <<" State: "<< getErrorInfo(errorCode));
			return false;
		}
	}
	return true;

}


EPOS::~EPOS() {
	setVelocity(0);
	nh.~NodeHandle();
}
