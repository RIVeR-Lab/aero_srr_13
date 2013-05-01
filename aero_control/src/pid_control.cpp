/*
 * pid_control.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: mdedonato
 */

#include <aero_control/pid_control.h>
using namespace pid;

PIDController::PIDController(float p, float i, float d, double error) {

	this->kp = p;
	this->ki = i;
	this->kd = d;
	prev_err = error;
	prev_err_time = ros::Time().now();
	old_err_counter = 0;
	old_err_position = 0;

	old_err[old_err_position] = error;
	old_err_counter++;
	old_err_position++;
}
void PIDController::SetP(float p) {
	this->kp = p;
}
void PIDController::SetI(float i) {
	this->ki = i;
}
void PIDController::SetD(float d) {
	this->kd = d;
}
void PIDController::SetPID(float p,float i,float d) {
	this->kp = p;
	this->ki = i;
	this->kd = d;
}

double PIDController::PIDUpdate(double error) {
	double output = 0;
	ros::Time current_time = ros::Time().now();
	double dt = current_time.toSec() - prev_err_time.toSec();

	output = Proportional(error) + Integral(error) + Derivative(error, dt);

	prev_err = error;
	prev_err_time = current_time;

	old_err[old_err_position] = error;
	old_err_position++;

	if (old_err_counter < ERROR_ARRAY_SIZE) {
		old_err_counter++;
	}

	if (old_err_position >= ERROR_ARRAY_SIZE) {
		old_err_position = 0;
	}

	return output;

}

double PIDController::Proportional(double error) {
	return kp * error;
}

double PIDController::Integral(double error) {
	double error_sum = 0;
	int i;
	for (i = 0; i < old_err_counter; i++) {
		error_sum = error_sum + old_err[i];
	}

	return ki * (error_sum + error);
}

double PIDController::Derivative(double error, double dt) {
	return kd * ((error - prev_err) / dt);
}
