#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
	Kp = Kp_init;
	Ki = Ki_init;
	Kd = Kd_init;
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
}

void PID::UpdateError(double cte) {
	d_error = p_error;
	p_error = cte;
	i_error += cte;
	std::cout << "p_err: " << p_error << "\ti_err: " << i_error << "\td_err: " << p_error - d_error << std::endl;
}

double PID::TotalError() {
	return -Kp*p_error - Ki*i_error - Kd*(p_error - d_error);
}

