# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

The objective of the project was to drive the car successfully in simulator provided by udacity using Model Predictive Control(MPC) .
MPC is an a way of controlling the vehicle in presence of constraints. MPC can follow different motion models. For this project I have used Kinetic model which is a simplified form of dynamic model as it ignores most of the forces like tyre force,gravity etc. .

By implementing this project i found that MPC is better as compared to PID in terms of the output. Though it turned out to be a bit hard to implement as compared to PID.

## Model

### State Variables 

* px: current location in x-axis of global coordinate system
* py: current location in y-axis of global coordinate system
* psi: current heading of vehicle
* v: current velocity of vehicle
* cte: diffrence between desired position and actual position
* epsi: diffrence between desired heading and actual heading

### Actuators

* delta: steering angle of the vehicle . It is in range from [-25,25]
* a : It determine the throttle and brake of the vehicle . Positive value of a determines 	 accelration/throttle and negative determines the break. Simulator accepts the value in range from [-1,1]

### Update Equations

![](https://github.com/singh-atul/CarND-MPC-Project/tree/master/img_res/image1.jpg)
![](https://github.com/singh-atul/CarND-MPC-Project/tree/master/img_res/image2.jpg)

dt = rate of change of state
lf = diffrenec between vehicle axle and its center of gravity


## Timestep Length and Elapsed Duration (N & dt)

* Time Step Length (N) : It determines how many states we need to look ahead in future.
* Elapsed Duration (dt) : Frequency we expect the state to change 

The time `T = N*dt` determines the prediction horizon . N imapcts on the performance of the controller i.e with a large value of N the controller starts to run slower and some time it also went off track very easily. And dt determines the latency, as an acctuation command in a real car wont execute instantly there will be a delay as it propogates through the system. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. So the value of the dt should be kept small. I tried the values of N from 7 to 20 and dt from 100 to 500 milliseconds out of which I decided to use `N = 10` and `dt = 100` milliseconds as they had a better result compared to other values in the simulator and the car was able to complete the lap with a speed in range of 30-45 mph successfully.




## Polynomial Fitting and MPC Preprocessing

I have used a 3rd degree polynomial as suggested in the classroom as it turned out to be the most appropriate one to get a smooth path throughout the lap.

The provided way-points are in global co-ordinate system so in order to convert the vehicle local co-ordinate system following formula are used : 

[img_res/image3]


Finally i have provided weight to the cost parameter . These weights are determ which parameter should be given more weightage. More the weight more penalty it will have on the cost error.

#### Cost weight for Refrence State 
* `CTE` : 2 
* `EPSI` : 500
* `Velocity` : 500

#### Cost for actuators

* `Delta` : 500
* `Acceleration` : 20 

#### Cost for value gap between sequential actuators

* `Delta`: 500 
* `Sequential`: 20

## Model Predictive Control with Latency


I have taken 100ms latency delay(dt) as it was in rubric of the project. In real case scenario the latency exists bewteen the commmand issue time and execution time. So inorder to compensate the latency , I have updated all the vehicle state with latency time before feeding it to the `MPC.Solve` function . These value are then fed to this function to predict the steer angle and throttle along with the x and y coordinates of the trajectory.




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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


