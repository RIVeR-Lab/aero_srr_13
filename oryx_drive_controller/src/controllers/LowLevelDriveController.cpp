/**
 * @file LowLevelDriveController.cpp
 *
 * @date Oct 7, 2012
 * @author Adam Panzica
 */

#include "LowLevelDriveController.h"

//*******************************Defines****************************/
#define FRONT_LEFT	0	///Location of the front left data in wheel data vector
#define FRONT_RIGHT 1	///Location of the front right data in wheel data vector
#define REAR_LEFT	2	///Location of the rear left data in wheel data vector
#define REAR_RIGHT	3	///Location of the rear right data in wheel data vector
#define SWERVE_OFF	4	///Location offset to swerve position data wheel data vector

//************************Local Function Protos*********************/
void printWheelVectorCalculation(std::vector<double>& wheelData);

//***************************Implementations************************/
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

void LowLevelDriveController::drive(double velocity, double radius){
	std::vector<double> wheelData(8);
	if(this->canSwerve){
		LowLevelDriveController::calculateSwerveTankSteer(velocity, radius, wheelData);
	}
	else{
		LowLevelDriveController::calculateTankSteer(velocity, radius, wheelData);
	}
}

void LowLevelDriveController::translate(double xVelocity, double yVelocity){
	std::vector<double> wheelData;
	if(this->canSwerve){
		LowLevelDriveController::calculateSwerveTankSteer(xVelocity, yVelocity, wheelData);
	}
}

void LowLevelDriveController::calculateTankSteer(double velocity, double radius, std::vector<double>& result){
	ROS_DEBUG("Calculating Tank-Steer on Parameters <V=%f, R=%f>", velocity, radius);
	//Initialize the return vector (FL, FR, RL, RR)
	result.resize(4);
	double leftWheelSpeed;
	double rightWheelSpeed;
	//Make checks for straight line/straight rotation
	if(radius == 0){
		leftWheelSpeed	= velocity;
		rightWheelSpeed	= -leftWheelSpeed;
	}
	else if(radius == std::numeric_limits<double>::infinity()){
		leftWheelSpeed	= velocity;
		rightWheelSpeed = leftWheelSpeed;
	}
	//Otherwise perform normal tank steer calculations
	else{
		double midpoint = this->baseWidth/2.0;
		double rv		= radius*velocity;
		double dv		= midpoint*velocity;

		//Calculate Left and Right Wheel Speeds
		leftWheelSpeed	= (rv-dv)/radius;
		rightWheelSpeed	= (rv+dv)/radius;
	}
	//Make one last check to make sure that numerical precision errors didn't result in infinities
	if((leftWheelSpeed == std::numeric_limits<double>::infinity())||(rightWheelSpeed == std::numeric_limits<double>::infinity())){
		//If radius was very small, probably was straight rotation, otherwise was probably straight line
		if(radius<1){
			leftWheelSpeed	= velocity;
			rightWheelSpeed	= -leftWheelSpeed;
		}else{
			leftWheelSpeed	= velocity;
			rightWheelSpeed	= leftWheelSpeed;
		}
	}
	//Form wheel velocity vector
	result.at(FRONT_LEFT)	= leftWheelSpeed;
	result.at(REAR_LEFT)	= result.at(FRONT_LEFT);
	result.at(FRONT_RIGHT)	= rightWheelSpeed;
	result.at(REAR_RIGHT)	= result.at(FRONT_RIGHT);
	printWheelVectorCalculation(result);
}

void LowLevelDriveController::calculateSwerveTankSteer(double velocity, double radius, std::vector<double>& result){
	ROS_DEBUG("Calculating Swerve Tank-Steer on Parameters <V=%f, R=%f>", velocity, radius);
	//Initialize the return vector (FL, FR, RL, RR, FLS, FRS, RLS, RRS)
	std::vector<double> wheelData(8,0);
	//TODO implement here
	printWheelVectorCalculation(wheelData);
}

void LowLevelDriveController::calculateSwerveTranslate(double xVelocity, double yVelocity, std::vector<double>& result){
	ROS_DEBUG("Calculating Swerve Translate on Parameters <XV=%f, YV=%f>", xVelocity, xVelocity);
	//Initialize the return vector (FL, FR, RL, RR, FLS, FRS, RLS, RRS)
	std::vector<double> wheelData(8,0);
	//TODO Implement here
	printWheelVectorCalculation(wheelData);
}

/**
 * Prints out the wheel data to ROS_INFO
 * @param wheelData Reference to a vector of wheel data to print
 */
void printWheelVectorCalculation(std::vector<double>& wheelData){
	//Check to see if there is swerve data
	bool swerveData = (wheelData.size()>SWERVE_OFF);
	//Print out wheel velocity vector
	ROS_INFO("Calculated Wheel Velocity Vector <FL=%f, FR=%f, RL=%f, RR=%f>", wheelData.at(FRONT_LEFT),wheelData.at(FRONT_RIGHT),wheelData.at(REAR_LEFT),wheelData.at(REAR_RIGHT));
	//Print out swerve position vector
	if(swerveData) ROS_INFO("Calculated Swerve Position Vector <FL=%f, FR=%f, RL=%f, RR=%f>", wheelData.at(FRONT_LEFT+SWERVE_OFF),wheelData.at(FRONT_RIGHT+SWERVE_OFF),wheelData.at(REAR_LEFT+SWERVE_OFF),wheelData.at(REAR_RIGHT+SWERVE_OFF));
}
