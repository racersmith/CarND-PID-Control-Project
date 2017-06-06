/*
* Twiddle
* Josh Smith
* 2017/05/30
*/
#include <iostream>
#include "Twiddle.h"

Twiddle::Twiddle() {}
Twiddle::~Twiddle() {}

void Twiddle::Init(unsigned long int error_time, unsigned long int wander_time, double wander_offset, double dp_p, double dp_i, double dp_d) {
	// Set wander parameters	
	this->cte_offset = wander_offset;
	this->error_time = error_time;
	this->wander_time = wander_time;
	
	// Set adjustment parameters
	dp[0] = dp_p;
	dp[1] = dp_i;
	dp[2] = dp_d;
	param_index = 0;

	// Offset scalar, setting this to 0 will turn off twiddle
	offset_state = 1;

	// Set best error to negative as flag for first pass
	running_error = 0.0;
	best_error = -1.0;

	// initialize test counter
	// negative values to add aditional time to first pass
	error_counter = -500;
	wander_counter = -500;

	// initialize twiddle_state
	twiddle_state = 0;
}

double Twiddle::RMSE() {
	double RMSE = std::sqrt(running_error);
	running_error = 0.0;
	return RMSE;
}

double Twiddle::Wander(double cte) {
	if (wander_time > 0) {
		wander_counter++;
		if (wander_counter >= wander_time) {
			wander_counter = 0;

			offset_state *= -1;
		}
		return cte + offset_state*cte_offset;
	}
	else {
		return cte;
	}
}

double Twiddle::Tune(double control_error, double& Kp, double& Ki, double& Kd) {
	// Check if tuning is complete
	double sum_dp = dp[0] + dp[1] + dp[2];
	
	if (sum_dp < 0.0001) {
		// return error unmodified to resume normal operation
		offset_state = 0;
		return control_error;
	}
	
	// Preturb the target position using wander
	double adjusted_control_error = Wander(control_error);

	// Accumulate squared error
	running_error += adjusted_control_error*adjusted_control_error;

	// Debug printing
	//std::cout << "wander: " << offset_state << "\error: " << adjusted_control_error << "\tSum dp: " << sum_dp << "\tKp: " << Kp << "\tKi: " << Ki ;
	//std::cout  << "\tKd: " << Kd  << "\tcounter: " << error_counter << "\tBest: " << best_error*best_error << "\tError: " << running_error << std::endl;
	
	// Check if evaluation time has elapsed or if the error has exceeded the best
	error_counter++;
	if (error_counter >= error_time || (running_error > 1.01 * best_error*best_error && best_error > 0.0)) {
		error_counter = 0;
		wander_counter = 0;

		double error = RMSE();

		double p[3];
		p[0] = Kp;
		p[1] = Ki;
		p[2] = Kd;
		const int n_param = 3;

		// On first pass update to actual error
		if (best_error == -1) {
			best_error = error;
			// start first test
			p[param_index] += dp[param_index];
		}
		// Improvement
		else if (error < best_error) {
			twiddle_state = 0;
			best_error = error;
			dp[param_index] *= 1.1;
			param_index = (param_index + 1) % n_param;
			p[param_index] += dp[param_index];
		}
		// No improvement
		else {
			switch (twiddle_state)
			{
			
			// After first parameter test
			case 0:
				twiddle_state = 1;
				// Check for calculation of negative param
				if (2.0*dp[param_index] < p[param_index]) {
					p[param_index] -= 2.0*dp[param_index];
				}
				else {
					p[param_index] = 0.25 * p[param_index];
				}
				break;

			// After second parameter test
			case 1:
				twiddle_state = 0;
				p[param_index] += dp[param_index];
				dp[param_index] *= 0.9;
				param_index = (param_index + 1) % n_param;
				p[param_index] += dp[param_index];

				std::cout << "\tKp: " << Kp << "\tKi: " << Ki << "\tKd: " << Kd << "Sum dp: " << sum_dp;
				std::cout << "\tBest: " << best_error*best_error << "\tError: " << running_error << std::endl;
				break;

			// What are you doing down here?
			default:
				twiddle_state = 0;
				param_index = 0;
				break;
			}
		}

		// Set new control parameters
		Kp = p[0];
		Ki = p[1];
		Kd = p[2];
	}

	return adjusted_control_error;
}
