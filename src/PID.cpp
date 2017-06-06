#include "PID.h"
#include "Twiddle.h"
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double lower_limit, double upper_limit) {
	this -> Kp = Kp;
	this -> Ki = Ki;
	this -> Kd = Kd;

	this->lower_limit = lower_limit;
	this->upper_limit = upper_limit;

	// Initialize errors
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;

	output = 0.0;
}

void PID::UpdateError(double error) {
	
	double K_total = Kp + Ki + Kd;

	// Reset integral error on parameter update
	if (K_total != Kp + Ki + Kd) {
		i_error = 0.0;
	}

	d_error = p_error;
	p_error = error;

	// Only accumulate i error if output is not railed
	// This is to avoid windup
	if (output <= upper_limit && output >= lower_limit) {
		i_error += error;
	}
}

double PID::Command() {
	output = -Kp*p_error - Ki*i_error - Kd*(p_error - d_error);

	// Keep within limits
	if (output > upper_limit) {
		output = upper_limit;
	}
	else if (output < lower_limit) {
		output = lower_limit;
	}

	return output;
}
