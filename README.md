# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## Windows Install Instructions 

1. Install, in your root `c:/` directory, vcpkg https://github.com/Microsoft/vcpkg   (15 - 30 minutes)
2. Be sure while installing vcpkg to carefully follow all instructions! This is NOT an easy install process.
3. Install python 2.7 (dependency for libuv) 
4. cd to directory with vcpkg .exe and `./vcpkg install uWebsockets` (20 min, mostly automatic)
5. Open CMakeSetting.json, check if `C:/vcpkg/scripts/buildsystems/vcpkg.cmake` is the correct directory to your vcpkg and `DCMAKE_TOOLCHAIN_FILE` matches the output from vcpkg integrate.
6. Open in VS17 community edition, build pid.exe in x86 debug.


## Rubric Points

The following section will cover the rubric points outlined for the project.

### Compilation

Project compiles using basic build instructions above.
[insert build example]

### Implementation

Outline PID controller implementation
#### PID
The code behind a PID controller is remarkably simple.  The controller is first initiallized with `PID::Init(Kp, Ki, Kd, lower_limit, upper_limit)`.  Where `Kp` is the proportional constant which controls how much of an impact any given error value impacts the control output, `Ki` is the integral constant which controls how much of an impact a sustained error value impacts the control output, and `Kd` is the derivative constant which controls how much the step-by-step change in error will impact the control output.
The `lower_limit` and `upper_limit` are used to constrain the controller's output values as well as stop the accumulation of integral error if the output is at a limit.  If this the integral error was allowed to accumulate when the output is at a maximum the integral error can quickly become extremely large and not allow the controller to recover.
After initialization the error is first updated by calling `PID::UpdateError(error)`, line 25, this updates the p, i and d error terms.  The control command can then be aquired by `PID::Command()`, line 44, which returns the control command by taking each of the p, i and d error terms and multiplying by their respective constants.

While the actual code behind the PID control the difficult part is selecting correct K parameters for the system.  To automate this process Twiddle is used.

#### Twiddle
Twiddle is a systematic method of  tuning hyperparameters.  Twiddle was implemented to tune the three PID parameters.  Each parameter goes through a tuning cycle where a slightly larger value is trialed and the system error is evaulated.  If the error is better than the step size for next time is increased and the next parameter is adjusted.  If the error does not improve the parameter is adjusted by the same amount in the opposite direction.  If this still does not provide an improved error then the step size is reduced.  
To improve tuning a forced perturbance is added to the system in the form of a squre wave.
Since the system was training while driving it was important to have reasonable initial tuning parameters such that the car would continue making laps around the test track.  After the step size for each parameter gets small enough Twiddle is turned off and normal driving operation returns.
The Twiddle implementation can be found in `Twiddle.cpp` and `Twiddle.h`.  

### Reflection
Describe the effect of the P, I and D compoents have.

Describe how hyperparameters were selected.  

### Simulation
Final solutions provides safe driving results on the test track.
[youtube video of result]