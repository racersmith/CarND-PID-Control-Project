/*
* Twiddle
* Josh Smith
* 2017/05/30
*/
#include <iostream>
#include "Twiddle.h"

Twiddle::Twiddle() {}
Twiddle::~Twiddle() {}

void Twiddle::Init(double offset, double trial_time, double dp_p, double dp_i, double dp_d) {
	cte_offset = offset;
	counter_max = trial_time;

	// Set adjustment parameters
	dp[0] = dp_p;
	dp[1] = dp_i;
	dp[2] = dp_d;
	param_index = 0;

	// Offset scalar, setting this to 0 will turn off twiddle
	offset_state = 1;

	// Set best error to negative as flag for first pass
	best_error = -1;

	// initialize test counter
	counter = 0;

	// initialize twiddle_state
	twiddle_state = 0;
}

double Twiddle::RMSE() {
	double RMSE = std::sqrt(running_error);
	running_error = 0.0;
	return RMSE;
}

double Twiddle::Tune(double cte, double& Kp, double& Ki, double& Kd) {
	// Check if tuning is complete
	double sum_dp = dp[0] + dp[1] + dp[2];
	
	std::cout << "Sum dp = " << sum_dp << std::endl;
	
	if (sum_dp < 0.001) {
		// return cte unmodified to resume normal driving
		offset_state = 0;
		return cte;
	}
	
	double adjusted_cte = cte + offset_state*cte_offset;
	// Add squared error to running error
	running_error += adjusted_cte*adjusted_cte;
	
	counter++;
	if (counter >= counter_max) {
		counter = 0;

		double error = RMSE();

		// create array
		double p[3];
		p[0] = Kp;
		p[1] = Ki;
		p[2] = Kd;

		std::cout << "Kp: " << Kp << "Ki: " << Ki << "Kd: " << Kd << std::endl;

		if (error < best_error) {
			best_error = error;
			dp[param_index] *= 1.1;
			twiddle_state = 0;
			param_index = (param_index + 1) % 3;
			p[param_index] += dp[param_index];
		}
		else {
			switch (twiddle_state)
			{
			case 0:
				twiddle_state = 1;
				p[param_index] -= 2 * dp[param_index];
				break;
			case 1:
				twiddle_state = 0;
				dp[param_index] *= 0.9;
				param_index = (param_index + 1) % 3;
				p[param_index] += dp[param_index];
				break;
			default:
				twiddle_state = 0;
				param_index = 0;
				break;
			}
		}

		// set new artificial target to expidite tunning
		offset_state *= -1;
		adjusted_cte = cte + offset_state*cte_offset;
	}
	return adjusted_cte;
}
