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

	steer_last = 0.0;

	// Tune parameters with Twiddle
	// Parameters: cte offset, measurement period, dp_p, dp_i, dp_d
	twiddle.Init(6000, 500, 0.5, 0.005, 1.0e-10, 0.5);
}

void PID::UpdateError(double cte) {
	
	double K_total = Kp + Ki + Kd;
	
	cte = twiddle.Tune(cte, Kp, Ki, Kd);

	// Reset integral error on parameter update
	if (K_total != Kp + Ki + Kd) {
		i_error = 0.0;
	}

	// Smooth cross track errors
	// seems to have discontinuities
	// This was can be used to apply a simple fitler
	// to ease initial tuning woes.
	//cte = d_error + 0.9*(cte - d_error);

	d_error = p_error;
	p_error = cte;

	// Only accumulate i error if output is not railed
	// This is to avoid windup
	if (steer_last <= upper_limit && steer_last >= lower_limit) {
		i_error += cte;
	}
}

double PID::TotalError() {
	double steer = -Kp*p_error - Ki*i_error - Kd*(p_error - d_error);
	steer_last = steer;

	// Keep within limits
	if (steer > upper_limit) {
		steer = upper_limit;
	}
	else if (steer < lower_limit) {
		steer = lower_limit;
	}

	return steer;
}
