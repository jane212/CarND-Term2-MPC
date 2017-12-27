# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Simulator Setting

* Screen resolution: 640x480
* Graphics quality: Fastest

## Descriptions

Some questions are answered and some details in the scripts are described below.

### Model

The model includes state, actuator and the update equations. As taught in the class, the state vector includes:

* `x`, `y`, `psi` and `v` for vehicle's location, orientation and speed;
* `cte` and `epsi` for the cross track error and orientation error.

The actuators are `delta` for steer and `a` for throttle. And the update equations are:

`x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt`

`y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt`

`psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt`

`v_[t+1] = v[t] + a[t] * dt`

`cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt`

`epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt`


Additionally, the cost is also evaluated by minimizing the `cte` and `epsi`. Also, a cost is put on actuators and their changes to make the control smooth. A weight of 500 is used for `delta` and its change.

### Timestep and Elapsed Duration

First tried `N=7` and `dt=0.05`, and the simulation showed the vehicle was not stable becasue the prediction steps were too short. After taking the quiz, finally decide to use `N=25` instead. The longer prediction steps perform better than previous. The elapsed duration is set to be 0.05 to make more accurate prediction and control.

### Polynomial Fitting and Preprocessing

Before fitting the waypoints, the data was tranformed from global coordinates to vehicle coordinates by:

`waypoints(0,i) =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);`

`waypoints(1,i) =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);`

### Latency

In the control setting, there is a latency of 100ms (0.1s). Since the elapsed duration is 0.05s, thus, I used 2 steps forward prediction value as the input of actuators.

