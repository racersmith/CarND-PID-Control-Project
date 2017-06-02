/*
* Twiddle
* Josh Smith
* 2017/05/30
*/
#include <iostream>
#include "Twiddle.h"

Twiddle::Twiddle() {}
Twiddle::~Twiddle() {}

void Twiddle::Init(double set_cte_offset, unsigned long int set_counter_max, unsigned long int set_settle_time, double dp_p, double dp_i, double dp_d) {
	cte_offset = set_cte_offset;
	error_time = set_counter_max;
	wander_time = set_settle_time;

	// Set adjustment parameters
	dp[0] = dp_p;
	//dp[1] = dp_i;
	//dp[2] = dp_d;
	dp[1] = dp_d;
	param_index = 0;

	// Offset scalar, setting this to 0 will turn off twiddle
	offset_state = 1;

	// Set best error to negative as flag for first pass
	running_error = 0;
	best_error = -1;

	// initialize test counter
	error_counter = 0;
	wander_counter = 0;

	// initialize twiddle_state
	twiddle_state = 0;
}

double Twiddle::RMSE() {
	double RMSE = std::sqrt(running_error);
	running_error = 0.0;
	return RMSE;
}

double Twiddle::Wander(double cte) {
	wander_counter++;
	if (wander_counter >= wander_time) {
		wander_counter = 0;
		offset_state *= -1;
	}
	return cte + offset_state*cte_offset;
}

double Twiddle::Tune(double cte, double& Kp, double& Ki, double& Kd) {
	// Check if tuning is complete
	double sum_dp = dp[0] + dp[1];// +dp[2];
	
	if (sum_dp < 0.0001) {
		// return cte unmodified to resume normal driving
		offset_state = 0;
		return cte;
	}
	
	double adjusted_cte = Wander(cte);
	// Add squared error to running error
	running_error += adjusted_cte*adjusted_cte;

	std::cout << "cte: " << adjusted_cte << "\tSum dp: " << sum_dp << "\tKp: " << Kp << "\tKi: " << Ki << "\tKd: " << Kd << "\tRun Err: " << running_error << std::endl;
	
	error_counter++;
	if (error_counter >= error_time) {
		error_counter = 0;

		double error = RMSE();

		// create array
		double p[2];
		p[0] = Kp;
		//p[1] = Ki;
		//p[2] = Kd;
		p[1] = Kd;

		int n_param = 2;

		// On first pass update to actual error
		if (best_error == -1) {
			best_error = error;
			// test first case
			p[param_index] += dp[param_index];
		}
		// On improvement
		else if (error < best_error) {
			best_error = error;
			dp[param_index] *= 1.1;
			twiddle_state = 0;
			param_index = (param_index + 1) % n_param;
			p[param_index] += dp[param_index];
		}
		// On no improvement
		else {
			switch (twiddle_state)
			{
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
			case 1:
				twiddle_state = 0;
				dp[param_index] *= 0.9;
				param_index = (param_index + 1) % n_param;
				p[param_index] += dp[param_index];
				break;
			default:
				twiddle_state = 0;
				param_index = 0;
				break;
			}
		}

		Kp = p[0];
		//Ki = p[1];
		//Kd = p[2];
		Kd = p[1];
	}
	return adjusted_cte;
}
