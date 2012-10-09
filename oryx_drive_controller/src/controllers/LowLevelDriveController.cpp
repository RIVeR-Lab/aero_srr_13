/**
 * @file LowLevelDriveController.cpp
 *
 * @date Oct 7, 2012
 * @author Adam Panzica
 */

#include "LowLevelDriveController.h"
#include <math.h>

//*******************************Defines****************************/
#define FRONT_LEFT	0	///Location of the front left data in wheel data vector
#define FRONT_RIGHT 1	///Location of the front right data in wheel data vector
#define REAR_LEFT	2	///Location of the rear left data in wheel data vector
#define REAR_RIGHT	3	///Location of the rear right data in wheel data vector
#define SWERVE_OFF	4	///Location offset to swerve position data wheel data vector

const double PI = std::atan(1.0)*4;	///Since C++ lacks a predefined PI constant, define it here

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

void LowLevelDriveController::setCanSwerve(bool canSwerve){
	this->canSwerve = canSwerve;
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

/**
 * Calculates the wheel velocities for all four wheels assuming standard tank-steering math model. Uses the following equations:
 * @f[ V_l =  \frac{(R \times V_c)-(M \times V_c)}{R} @f]
 * @f[ V_r =  \frac{(R \times V_c)+(M \times V_c)}{R} @f]
 *
 * Where @f$ V_l @f$ is the linear velocity of the left-side wheels, and
 * @f$ V_r @f$ is the linear velocity of the right-side wheels
 */
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

/**
 * Calculates wheel speeds and swerve positions based on the following formulae:
 * @f[ R_si = R_c - W_m @f]
 * @f[ R_so = R_c + W_m @f]
 * @f[ R+wi = \sqrt{L_m ^2 + R_si ^2} @f]
 * @f[ R+wo = \sqrt{L_m ^2 + R_so ^2} @f]
 * @f[ theta_i = \arctan 2(L_m,R_si) @f]
 * @f[ theta_o = \arctan 2(L_m,R_so) @f]
 * @f[ V_wi = V_c \times \frac{R_wi}{R_c} @f]
 * @f[ V_wo = V_c \times \frac{R_wo}{R_c} @f]
 *
 * Where @f$ V_wi @f$ is the linear velocity of the inside wheels,
 * @f$ V_wi @f$ is the linear velocity of the inside wheels,
 * @f$ \plusminus theta_i @f$ is the swerve position of the inside wheels, and
 * @f$ \plusminus theta_o @f$ is the swerve position of the outside wheels
 */
void LowLevelDriveController::calculateSwerveTankSteer(double velocity, double radius, std::vector<double>& result){
	ROS_DEBUG("Calculating Swerve Tank-Steer on Parameters <V=%f, R=%f>", velocity, radius);
	//Initialize the return vector (FL, FR, RL, RR, FLS, FRS, RLS, RRS)
	result.resize(8);
	//Speeds and Swerves
	double leftFrontWheelSpeed;
	double leftRearWheelSpeed;
	double rightFrontWheelSpeed;
	double rightRearWheelSpeed;
	double leftFrontSwerve;
	double rightFrontSwerve;
	double leftRearSwerve;
	double rightRearSwerve;

	//Check for the simple cases (straight line, straight rotation)
	if(radius==0){
		leftFrontWheelSpeed  = velocity;
		rightFrontWheelSpeed = velocity;
		leftRearWheelSpeed	 = velocity;
		rightRearWheelSpeed	 = velocity;

		double theta = PI/4;
		leftFrontSwerve		= theta;
		rightFrontSwerve	= -theta;
		leftRearSwerve		= -theta;
		rightRearSwerve		= theta;
	}
	else if(radius==std::numeric_limits<double>::infinity()){
		leftFrontWheelSpeed  = velocity;
		rightFrontWheelSpeed = velocity;
		leftRearWheelSpeed	 = velocity;
		rightRearWheelSpeed	 = velocity;

		leftFrontSwerve		= 0;
		rightFrontSwerve	= 0;
		leftRearSwerve		= 0;
		rightRearSwerve		= 0;
	}
	else{

		//Calculation varables
		double width_midpoint 			= this->baseWidth/2.0;
		double length_midpoint			= this->baseLength/2.0;
		double radius_center_inside		= radius-width_midpoint;
		double radius_center_outside	= radius+width_midpoint;
		double radius_wheel_inside		= std::sqrt(std::pow(length_midpoint,2)+std::pow(radius_center_inside,2));
		double radius_wheel_outside		= std::sqrt(std::pow(length_midpoint,2)+std::pow(radius_center_outside,2));
		double theta_inside				= std::atan2(length_midpoint,radius_center_inside);
		double theta_outside			= std::atan2(length_midpoint,radius_center_outside);
		double radius_ratio_inside		= radius_wheel_inside/radius;
		double radius_ratio_outside		= radius_wheel_outside/radius;


		rightFrontWheelSpeed = velocity*radius_ratio_inside;
		rightRearWheelSpeed	 = rightFrontWheelSpeed;
		leftFrontWheelSpeed  = velocity*radius_ratio_outside;
		leftRearWheelSpeed	 = leftFrontWheelSpeed;

		rightFrontSwerve	= theta_inside;
		rightRearSwerve		= -theta_inside;
		leftFrontSwerve		= theta_outside;
		leftRearSwerve		= -theta_outside;

	}
	//results
	result.at(FRONT_LEFT)				= leftFrontWheelSpeed;
	result.at(REAR_LEFT)				= leftRearWheelSpeed;
	result.at(FRONT_RIGHT)				= rightFrontWheelSpeed;
	result.at(REAR_RIGHT)				= rightRearWheelSpeed;
	result.at(FRONT_LEFT+SWERVE_OFF)	= leftFrontSwerve;
	result.at(REAR_LEFT+SWERVE_OFF)		= rightFrontSwerve;
	result.at(FRONT_RIGHT+SWERVE_OFF)	= leftRearSwerve;
	result.at(REAR_RIGHT+SWERVE_OFF)	= rightRearSwerve;


	printWheelVectorCalculation(result);
}

void LowLevelDriveController::calculateSwerveTranslate(double xVelocity, double yVelocity, std::vector<double>& result){
	ROS_DEBUG("Calculating Swerve Translate on Parameters <XV=%f, YV=%f>", xVelocity, xVelocity);
	//Initialize the return vector (FL, FR, RL, RR, FLS, FRS, RLS, RRS)
	result.resize(8);
	double leftFrontWheelSpeed;
	double rightFrontWheelSpeed;
	double leftRearWheelSpeed;
	double rightRearWheelSpeed;

	double leftFrontSwerve;
	double rightFrontSwerve;
	double leftRearSwerve;
	double rightRearSwerve;
	//TODO Implement here

	//Form wheel velocity vector
	result.at(FRONT_LEFT)	= leftFrontWheelSpeed;
	result.at(REAR_LEFT)	= leftRearWheelSpeed;
	result.at(FRONT_RIGHT)	= rightFrontWheelSpeed;
	result.at(REAR_RIGHT)	= rightRearWheelSpeed;
	printWheelVectorCalculation(result);
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
