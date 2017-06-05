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

  PID steering;
	//Initialize pid controller Kp, Ki, Kd, lower_limit, upper_limit
	steering.Init(0.3145, 1.47e-9, 5.84, -1, 1);

	
	// Tune parameters with Twiddle
	Twiddle steerTune;
	// Parameters: cte offset, measurement period, dp_p, dp_i, dp_d
	steerTune.Init(6000, 500, 0.0, 0.03, 1.0e-5, 0.5);

	const double TARGET_SPEED = 50;
	PID throttle;
	//Initialize pid controller Kp, Ki, Kd, lower_limit, upper_limit
	throttle.Init(2.25, 0.0166, 1.92, -1, 1);

	Twiddle throttleTune;
	throttleTune.Init(600, 200, 2.0, 0.2, 0.001, 0.1);
  

  h.onMessage([&steering, &steerTune, &throttle, &throttleTune, &TARGET_SPEED](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

					/*
					* Steering Control
					*/

					// Tune steering PID parameters with twiddle
					cte = steerTune.Tune(cte, steering.Kp, steering.Ki, steering.Kd);
				
					// Update steering error and command steering
					steering.UpdateError(cte);
					double steer_value = steering.Command();

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
					
					/*
					* Throttle Control
					*/

					double error_speed = speed - TARGET_SPEED;
					
					// Tune Throttle PID parameters with twiddle
					error_speed = throttleTune.Tune(error_speed, throttle.Kp, throttle.Ki, throttle.Kd);

					// Update throttle error and command throttle
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