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
	int error_counter;
	int error_time;

	/*
	* Cross track error offset
	*/
	double cte_offset;
	int offset_state;
	int wander_counter;
	int wander_time;

	/*
	* Calculate the total PID error.
	*/
	double RMSE();
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

	* @return Preturbed CTE from wander
	*/
	double Tune(double cte, double& Kp, double& Ki, double& Kd);
};

#endif /* Twiddle_H */
