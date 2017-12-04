# Model Predicitve Control (MPC) Project 


| **Source Code**  			| [https://github.com/aurangzaib/CarND-MPC-Project](https://github.com/aurangzaib/CarND-MPC-Project) |
|:-------------------------|:-------------|
| **Overview**  				| `README.md`  |
| **Setup**  					| `SETUP.md`  |
|**MPC Hyperparameters**	| `src/mpc.h`|
|**MPC Implementation**		| `src/mpc.cpp`|
| **How to run**  			| `mkdir build && cd build` | 
| 								| `cmake .. && make`|
| 								| `./mpc`|


## Introduction:

Model Predictive Control is used to estimate optimized `acceleration`, `steering` and `cross track error` values with minimum cost using `position` and `velocity`.

MPC works better than PID controller as it can anticipate future events and take control accordingly. It also deals with actuation delays better than PID controller.

The steps of the project are as following:

- Read `px, py ` and `v` from simulator

- Initialize MPC

- Define Vehicle Model using Global Kinematic Model

- Define Constraints

- Define Cost function

- Run State Feedback Loop and call Optimzation Solver passing it the current state

## Explanation of the code:

The implementation of MPC Project is divided into 2 files:

`main.cpp`

`mpc.cpp`

Following table summarizes the purpose of each file:

| File | Explanation |
|:-----------|:-------------|
|**main.cpp**| |
|| Get position (px, py) and velocity (v) for simulator |
||Find waypoints by transforming position fom Map coordinate system to Vehicle coordinate system|
||Find 3rd polynomial coefficients|
||Use Global Kinematic Model to caculate initial State (`px, py, psi, v, cte, epsi`) values|
||Call `Solve` method of class `MPC`|
||Send Actuations (acceleration and steering) and State (`px, py`) to simulator |
|**mpc.cpp**| |
||Define hyper parameters of MPC|
||Define start array positions of `States`, `Errors` and `Actuations` (control inputs) for vars and constraints (fg)|
||Calculate initial cost for errors. It uses Cross track error, Orientation error and Velocity error|
||Calculate initial cost for control inputs. It uses acceleration and steering angle|
||Calculate initial cost for sequential smoothing. It uses difference of acceleration and steering angle with respective previous values|
||Set State and errors in contraints. Control input is not part of constraints|
|| Set lower and upper boundaries for vars and constraints|
||Call `solve` method of `Ipopt` which returns Cost, optimized Actuations and State values |
||Return Actuations (acceleration and steering) and State (`px, py`) to main function|


## Hyperparameters in MPC:

In general, smaller dt gives better accuracy but that will require higher N for given horizon (N*dt). However, increase N will result in longer computational time which effectively increase the latency.

Following hyperparemters are used in MPC:

| Param | Symbol | Value |
|:-----------|:-------------|:-------------|
|Number of actuations|`N`|12|
|Time elapsed between actuations|`dt`|0.1|
|Distance from front wheel to CoG|`Lf`|2.67|
|Reference velocity|`v_ref`|110|
|CTE cost weight|`weight_cte`|12e3|
|EPSI cost weight|`weight_epsi`|12e2|
|Velocity cost weight|`weight_v`|1|
|Steering cost weight|`weight_delta`|12e2|
|Change in steering cost weight|`weight_delta_change`|12|
|Acceleration cost weight|`weight_a`|12|
|Change in acceleration cost weight|`weight_a_change`|12|

## Results


Following points sum up the results and conclusion for MPC:

- Calculation of actuations using MPC allows to drive a car much better compared to PID controllers.

- There is less oscillation in car movements and acceleration profile.

- As MPC is mathematically more complex than PID controller, different parameters can be controlled using cost function weights e.g. CTE, steering, orientation weights etc.

- Latency can be better handled in MPC than PID controller.

The video (GIF) below shows results of MPC:


![Results](result-mpc.gif)