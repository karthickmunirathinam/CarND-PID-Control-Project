# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

### Rubric

#### Compilation

1. Your code should compile.

Based on the steps provided above, following output was received for build.
```
karthick@karthick-HP-ProBook-430-G5:~/Documents/Udacity/pid_control/CarND-PID-Control-Project/build$ cmake ..
-- Configuring done
-- Generating done
-- Build files have been written to: /home/karthick/Documents/Udacity/pid_control/CarND-PID-Control-Project/build
karthick@karthick-HP-ProBook-430-G5:~/Documents/Udacity/pid_control/CarND-PID-Control-Project/build$ make
Scanning dependencies of target pid
[ 33%] Building CXX object CMakeFiles/pid.dir/src/main.cpp.o
/home/karthick/Documents/Udacity/pid_control/CarND-PID-Control-Project/src/main.cpp: In lambda function:
/home/karthick/Documents/Udacity/pid_control/CarND-PID-Control-Project/src/main.cpp:58:18: warning: unused variable ‘speed’ [-Wunused-variable]
           double speed = std::stod(j[1]["speed"].get<std::string>());
                  ^
/home/karthick/Documents/Udacity/pid_control/CarND-PID-Control-Project/src/main.cpp:59:18: warning: unused variable ‘angle’ [-Wunused-variable]
           double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                  ^
[ 66%] Linking CXX executable pid
[100%] Built target pid
```

#### Implementation

1. The PID procedure follows what was taught in the lessons.

The modified files are `src/main.cpp`, `src/PID.h` and `src/PID.cpp`
For ``PID`` features and error calculations implementation is part of `src/PID.cpp`.
The class is implemented in `src/PID.h`.
The `src/main.cpp` is modified in the TODO part where the PID parameter initialization and throtle is modified based on the steering value.

#### Reflection

1. Describe the effect each of the P, I, D components had in your implementation.
-  Proportional portion(Kp) of controller tries to steer the car towards the center line. If the proportional gain is set high the controller overshoots the centre line and starts to oscillate. This can be observed when the car constantly overcorrect and overshoot the middle. If the gain is set too low the controller reacts slowly to the error.
-  Integral portion(Ki) of controller tries eliminate possible bias and drifts from the center line. Larger gain has led to oscillation.
-  Differential portion(Kd) of controller tries to reduce the overshoot caused by the proportional component. This a basically a damper, so larger gain leads to slow reduction of cross track error and higher value leads to overshoot and oscillation.

2. Describe how the final hyperparameters were chosen.
-  Parameters were chosen manually with trial and error.
-  Initially set all the gains Kp, Kv and Ki to zeros. The car now drives in a staright line.
-  Increase Kp untill the car set to oscillate. Then reduce the Kp value to half of its value.
-  Now increase the Kd gain so the overshoot of center line had reduced.
-  Finally set a small value of Ki gain in small steps to reduce the drifts and bias in the turns.
-  P: 0.25  I: 0.003  D: 3

3. Effects of Throttle
-  It was abolutely necessary to control the speed especially during the turns. The the speed of the car is controlled with the throttle parameter. It was switched between 0 and 1. Largeer steering angle (during sharp turns) means lesser throttle. I have tuned the value of multiplying factor so the speed is set within a safe limits in all possible turn angles.

#### Simulation

1. The vehicle must successfully drive a lap around the track.

   Vehicle has successfully completed lap without going out of the road with set parameters.