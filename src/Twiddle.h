/* 
* Twiddle
* Josh Smith
* 2017/05/30
*/

#ifndef Twiddle_H
#define Twiddle_H

class Twiddle {
private:
	/*
	* parameter step size
	*/
	double dp[3];
	int param_index;

	/*
	* Twiddle state
	*/
	int twiddle_state;

	/*
	* error
	*/
	double running_error;
	double best_error;
	
	/*
	* Errror measurement time
	*/
	unsigned long int counter;
	unsigned long int counter_max;

	/*
	* Cross track error offset
	*/
	double cte_offset;
	int offset_state;

	/*
	* Calculate the total PID error.
	*/
	double RMSE();


public:
	/*
	* Constructor
	*/
	Twiddle();

	/*
	* Destructor.
	*/
	virtual ~Twiddle();

	/*
	* Initialize PID.
	*/
	void Init(const double offset, const double trial_time, double dp_p, double dp_i, double dp_d);
	
	/*
	* Tune the PID paramters
	*/
	double Tune(double cte, double& Kp, double& Ki, double& Kd);
};

#endif /* Twiddle_H */
