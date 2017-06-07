#define _USE_MATH_DEFINES

#include "uWS/uWS.h"
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "Twiddle.h"



using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::stringstream hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return std::stringstream();
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    std::stringstream tmp = std::stringstream();
    tmp.str(s.substr(b1, b2 - b1 + 1));
    return tmp;
  }
  return std::stringstream();
}


int main()
{

  uWS::Hub h;


	//Initialize pid controller
	double steer_Kp = 0.345259;
	double steer_Ki = 4.91032e-08;
	double steer_Kd = 6.24905;
	double steer_lower_limit = -1.0;
	double steer_upper_limit = 1.0;
	PID steering;
	steering.Init(steer_Kp, steer_Ki, steer_Kd, steer_lower_limit, steer_upper_limit);

	// Steering tuning with Twiddle
	Twiddle steerTune;
	// Parameters: cte offset, measurement period, dp_p, dp_i, dp_d
	steerTune.Init(6000, 500, 0.0, 0.2*steer_Kp, 0.2*steer_Ki, 0.2*steer_Kd);

	// Speed is a function of steering angle
	// These parameters set the max and min
	// speeds based on steering.
	const double MAX_SPEED = 50;
	const double MIN_SPEED = 10;
	
	PID throttle;
	//Initialize pid controller Kp, Ki, Kd, lower_limit, upper_limit
	double throttle_Kp = 3.0104;
	double throttle_Ki = 0.00843383;
	double throttle_Kd = 2.29189;
	double throttle_lower_limit = -1.0;
	double throttle_upper_limit = 1.0;
	throttle.Init(throttle_Kp, throttle_Ki, throttle_Kd, throttle_lower_limit, throttle_upper_limit);

	// Throttle tuning with Twiddle
	Twiddle throttleTune;
	throttleTune.Init(600, 200, 5.0, 0.2*throttle_Kp, 0.2*throttle_Ki, 0.2*throttle_Kd);
  

  h.onMessage([&steering, &steerTune, &throttle, &throttleTune, &MAX_SPEED, &MIN_SPEED](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s.str() != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

					/*
					* Steering Control
					*/
					// Tune steering PID parameters with twiddle
					if (false) {
						cte = steerTune.Tune(cte, steering.Kp, steering.Ki, steering.Kd);
					}
				
					// Update steering error and command steering
					steering.UpdateError(cte);
					double steer_value = steering.Command();

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
					
					/*
					* Throttle Control
					*/
					// Set throttle as a function of steering angle
					// 1st Order
					//double adjusted_speed_target = -(MAX_SPEED - MIN_SPEED) * std::abs(steer_value) + MAX_SPEED;
					// 2nd Order
					double adjusted_speed_target = (MAX_SPEED - MIN_SPEED) * (1.0 - steer_value*steer_value) + MIN_SPEED;

					double error_speed = speed - adjusted_speed_target;
					
					// Tune Throttle PID parameters with twiddle
					if (false) {
						error_speed = throttleTune.Tune(error_speed, throttle.Kp, throttle.Ki, throttle.Kd);
					}
					// Update throttle error and commanded speed
					throttle.UpdateError(error_speed);
					double long_pedal = throttle.Command();

					// DEBUG
					//std::cout << "Speed Error: " << error_speed << " Throttle Value: " << long_pedal << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = long_pedal;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          (ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        (ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    (ws).close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}