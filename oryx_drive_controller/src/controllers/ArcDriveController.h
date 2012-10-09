/**
 * @file ArcDriveController.h
 *
 * @date Oct 7, 2012
 * @author Adam Panzics
 */

#ifndef ARCDRIVECONTROLLER_H_
#define ARCDRIVECONTROLLER_H_

#include <ros/ros.h>
#include <OryxMessages/WheelVelocities.h>
#include <OryxMessages/SwervePositions.h>
/**
 * @brief Low level velocity-arc based drive controller
 *
 * Implements a velocity-control based controller which is capable of performing arc based motions. There are separate
 * control modes for operation based on the capabilities of the base platform:
 * *Without Swerve Capabilities
 *  -Standard Tank Steer Velocity-Arc
 * *With Swerve Capabilities
 *  -Smooth Velocity-Arc
 *  -Translation
 */
class ArcDriveController{
public:
	/**
	 * Creates a new ArcDriveController with standard initialization parameters
	 * @param drive_velocity_topic		The topic name to publish wheel velocity messages to
	 * @param drive_swerve_topic		The topic name to publish swerve position messages to
	 * @param drive_capabilities_topic	The topic name to poll for DriveManager capabilities
	 * @param baseLength				The length of the base platform from front wheel center to rear wheel center
	 * @param baseWidth					The width of the base platform from left wheel center to right wheel center
	 */
	ArcDriveController(std::string drive_velocity_topic,
					std::string drive_swerve_topic,
					std::string drive_capabilities_topic,
					double baseLength,
					double baseWidth);
	///Default destructor
	virtual ~ArcDriveController();

	/**
	 * @brief Tells the controller to perform a velocity controlled arc-driving motion
	 * @param velocity Linear velocity to maintain
	 * @param radius Radius of the arc to traverse (float.MAX will result in a straight line)
	 * Note that the controller assumes that right-handed radii are positive. It also assumes that the robot frame is aligned
	 * such that the +X axis is in the center-forward position.
	 */
	void drive(double velocity, double radius);

	/**
	 * @brief Tells the controller to perform a velocity controlled translate motion
	 * @param xVelocity The X velocity to maintain (relative to robot)
	 * @param xVelocity The Y velocity to maintain (relative to robot)
	 * Note that the controller assumes that right-handed radii are positive. It also assumes that the robot frame is aligned
	 * such that the +X axis is in the center-forward position.
	 */
	void translate(double xVelocity, double yVelocity);

	/**
	 * Tells the controller whether the platform is swerve capable or not
	 * @param canSwerve TRUE for swerve, else FALSE
	 */
	void setCanSwerve(bool canSwerve);

private:
	bool	canSwerve;				///Flag for signaling if swerve control is possible
	double	baseLength;				///The length of the platform
	double 	baseWidth;				///The width of the platform
	ros::NodeHandle nh;				///Node handle into the ROS system
	ros::Publisher velocity_pub;	///Publisher for sending velocity messages to DriveManager
	ros::Publisher swerve_pub;		///Publisher for sending swerve message to DriveManager

	/**
	 * @brief Calculates the wheel velocities used for standard, non-swerve tank steering
	 * @param velocity The linear velocity for traversing the arc
	 * @param radius The radius of the arc to traverse
	 * @param result Reference to a vector to write the result to, which will be  front-left, front-right, rear-left, rear-right wheel velocities in that order.
	 */
	void calculateTankSteer(double velocity, double radius, std::vector<double>& result);

	/**
	 * @brief Calculates the wheel velocities used for swerve-based tank steering
	 * @param velocity
	 * @param radius
	 * @param result Reference to a vector to write the result to, which will be  front-left, front-right, rear-left, rear-right wheel velocities in that order.
	 */
	void calculateSwerveTankSteer(double velocity, double radius, std::vector<double>& result);

	/**
	 * @brief Calculates the wheel velocities and swerve positions to translate
	 * @param xVelocity X Velocity to maintain during the translate
	 * @param yVelocity Y Velocity to maintain during the translate
	 * @param result Reference to a vector to write the result to, which will be  front-left, front-right, rear-left, rear-right wheel velocities in that order.
	 */
	void calculateSwerveTranslate(double xVelocity, double yVelocity, std::vector<double>& result);

	/**
	 * Initializaiton function which checks if the platform can swerve
	 */
	void checkCapabilities();
};

#endif /* ARCDRIVECONTROLLER_H_ */
