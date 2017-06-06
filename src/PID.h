#ifndef PID_H
#define PID_H
#include "Twiddle.h"

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
	// Proportional constant
  double Kp;
	// Integral constant
  double Ki;
	// Derivative constant
  double Kd;
	// Smallest output value
	double lower_limit;
	// Largest output value
	double upper_limit;
	// Last output command
	double output;

	/*
	* Twiddle tune
	*/
	Twiddle twiddle;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
	* @param Kp Proportional constant
	* @param Ki Integral constant
	* @param Kd Derivative constant
	* @param lower_limit Smallest allowed output
	* @param upper_limit Largest allowed output
  */
  void Init(double Kp, double Ki, double Kd, double lower_limit, double upper_limit);

  /*
  * Update the PID error variables given cross track error.
	* @param cte Cross track Error
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
	* @return Control command
  */
  double Command();
};

#endif /* PID_H */
