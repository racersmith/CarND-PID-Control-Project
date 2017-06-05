/* 
* Twiddle
* Josh Smith
* 2017/05/30
*/

#ifndef Twiddle_H
#define Twiddle_H

/*
* Twiddle
* Systematically adjust parameters to minimize error
*/
class Twiddle {
private:
	// parameter step size
	double dp[3];
	// Current adjustment parameter
	int param_index;

	// Current parameter trial state
	int twiddle_state;

	// Cumulative error of current parameter trial
	double running_error;
	// Lowest achieved error
	double best_error;
	
	// Counter to track current trial time
	int error_counter;
	// How long to test a parameter set
	int error_time;

	// How much to offset from centerline when wandering
	double cte_offset;
	// Which side of center to offset
	int offset_state;
	// Counter to track current wander cycle
	int wander_counter;
	// How long to wander to one side of the road
	int wander_time;

	/*
	* Calculate the total PID error.
	*/
	double RMSE();

	/*
	* Wander
	* Artificial forced preturbance
	* @param cte Actual Cross Track Error
	*
	* @return Preturbed Cross Track Error
	*/
	double Wander(double cte);

	


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
	* Initialize Twiddle
	* 
	* @param error_time How long to trial a new parameter set
	* @param wander_time How frequently to switch sides of road
	* @param wander_offset How much to shift off centerline when wandering
	* @param dp_p Initial adjustment size of Kp
	* @param dp_i Initial adjustment size of Ki
	* @param dp_d Initial adjustment size of Kd
	*/
	void Init(unsigned long int error_time, unsigned long int wander_time, double wander_offset, double dp_p, double dp_i, double dp_d);
	
	/*
	* Tune the PID paramters
	* 
	* @param cte Current cross track error
	* @param Kp Current Kp parameter
	* @param Ki Current Ki parameter
	* @param Kd Current Kd parameter
	*
	* @return Preturbed CTE from wander
	*/
	double Tune(double cte, double& Kp, double& Ki, double& Kd);
};

#endif /* Twiddle_H */
