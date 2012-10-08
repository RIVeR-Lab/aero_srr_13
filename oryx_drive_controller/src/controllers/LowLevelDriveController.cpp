/**
 * @file LowLevelDriveController.cpp
 *
 * @date Oct 7, 2012
 * @author Adam Panzica
 */

#include "LowLevelDriveController.h"

#define FRONT_LEFT	0	///Location of the front left data in wheel data vector
#define FRONT_RIGHT 1	///Location of the front right data in wheel data vector
#define REAR_LEFT	2	///Location of the rear left data in wheel data vector
#define REAR_RIGHT	3	///Location of the rear right data in wheel data vector
#define SWERVE_OFF	4	///Location offset to swerve position data wheel data vector

//Function Protos
void printWheelVectorCalculation(std::vector<double> wheelData);

LowLevelDriveController::LowLevelDriveController(std::string drive_velocity_topic,
												 std::string drive_swerve_topic,
												 std::string drive_capabilities_topic,
												 double baseLength,
												 double baseWidth) {
	ROS_INFO("Starting Up Low Level Drive Controller");
	//TODO actually set up correct message publishing data
	//Set up publisher to the DriveManager wheel velocity topic
	this->velocity_pub = nh.advertise<std_msgs::String>(drive_velocity_topic.c_str(),2);
	//Set up publisher to the DriveManager swerve control topic
	this->swerve_pub = nh.advertise<std_msgs::String>(drive_swerve_topic.c_str(),2);
	//set up dimensional parameters
	this->baseLength = baseLength;
	this->baseWidth  = baseWidth;
	this->canSwerve  = false;
	ROS_DEBUG("Got a platform size of <L=%f,W=%f>",baseLength, baseWidth);
}

LowLevelDriveController::~LowLevelDriveController() {
	// TODO Auto-generated destructor stub
}

std::vector<double> LowLevelDriveController::calculateTankSteer(double velocity, double radius){
	ROS_DEBUG("Calculating Tank-Steer on Parameters <V=%f, R=%f>", velocity, radius);
	//Initialize the return vector (FL, FR, RL, RR)
	std::vector<double> wheelVelocities(4,0);
	//TODO Implement here
	printWheelVectorCalculation(wheelVelocities);
	return wheelVelocities;
}

std::vector<double> LowLevelDriveController::calculateSwerveTankSteer(double velocity, double radius){
	ROS_DEBUG("Calculating Swerve Tank-Steer on Parameters <V=%f, R=%f>", velocity, radius);
	//Initialize the return vector (FL, FR, RL, RR, FLS, FRS, RLS, RRS)
	std::vector<double> wheelData(8,0);
	//TODO Implement here
	printWheelVectorCalculation(wheelData);
	return wheelData;
}

std::vector<double> LowLevelDriveController::calculateSwerveTranslate(double xVelocity, double yVelocity){
	ROS_DEBUG("Calculating Swerve Translate on Parameters <XV=%f, YV=%f>", xVelocity, xVelocity);
	//Initialize the return vector (FL, FR, RL, RR, FLS, FRS, RLS, RRS)
	std::vector<double> wheelData(8,0);
	//TODO Implement here
	printWheelVectorCalculation(wheelData);
	return wheelData;
}

void printWheelVectorCalculation(std::vector<double> wheelData){
	//Check to see if there is swerve data
	bool swerveData = (wheelData.size()>SWERVE_OFF);
	//Print out wheel velocity vector
	ROS_INFO("Calculated Wheel Velocity Vector <FL=%f, FR=%f, RL=%f, RR=%f>", wheelData.at(FRONT_LEFT),wheelData.at(FRONT_RIGHT),wheelData.at(REAR_LEFT),wheelData.at(REAR_RIGHT));
	//Print out swerve position vector
	if(swerveData) ROS_INFO("Calculated Swerve Position Vector <FL=%f, FR=%f, RL=%f, RR=%f>", wheelData.at(FRONT_LEFT+SWERVE_OFF),wheelData.at(FRONT_RIGHT+SWERVE_OFF),wheelData.at(REAR_LEFT+SWERVE_OFF),wheelData.at(REAR_RIGHT+SWERVE_OFF));
}
