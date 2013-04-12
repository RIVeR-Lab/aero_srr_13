/*
 * pid_control.h
 *
 *  Created on: Apr 11, 2013
 *      Author: mdedonato
 */

#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#include <ros/ros.h>



namespace pid
{

#define ERROR_ARRAY_SIZE 100

class PIDController
{
public:
	PIDController(float p, float i, float d, double error);
	~PIDController();

	double PIDUpdate(double error);
private:
	double Proportional(double error);
	double Integral(double error);
	double Derivative(double error, double dt);

	float kp;
	float ki;
	float kd;
	double prev_err;
	ros::Time prev_err_time;
	double old_err[ERROR_ARRAY_SIZE];

	int old_err_counter;
	int old_err_position;

};
}

#endif /* PID_CONTROL_H_ */
