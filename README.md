# Model Predictive Control Project
Implementation of a MP Controller for the Self-Driving Car Engineer Nanodegree Program. 

A `MPC` helps fix the problem we had with the `PID` controller where it is reacting to events
that happened in the past. Alternatively, the `MPC` will make predictions about future trajectory 
of the vehicle.

---

# Strategy 
The goal of the project was to reach high speed. To achieve this goal, a combination of steering
minimizing, steering smoothing and velocity management was utilized. In addition Latency was
was accounted for by calculating the time delta and performing a calculation with a `Kitmatic Model`
to produce the starting point for the solver.

## Motion Model

The `Kitematic` motion model was used due to its simplicity and performance 
considering it does not incorporate the g-forces that the more complex 
dynamic models capture. Since it doesn't capture these g-forces it is better
for lower speeds. Velocity management was introduced to regulate speed in 
situations where error is not manageable. 

## State

The state used in the motion model consisted of the following 6 variables:

- x coordinate
- y coordinate
- psi or orientation
- velocity
- cross track error
- error psi or orientation error

## Actuators

The MPC makes predictions for both `velocity` and `steering angle`.

## Lookahead time

The MPC will predict a series of discrete points, which will be plotted as a 
`green` line in the simulator. During testing the time delta between the 
points seemed be best at `.05` maximize number of points within the small
lookahead window. `15` points where chosen to give enough lookahead time
for high speeds.

## Fit Reference Waypoints

To have a polynomial to reference we fit a polynomial to the waypoints. Since,
these waypoints are in the global coordinate system, the waypoints are
preprocessed into the vehicle coordinate system prior to fitting the polynomial.  
     
## Velocity Management

Velocity was adjusted every message depending on the `Cross Track Error`. This allowed for higher
speeds in the straights and the vehicle could take extra care in the corners.

## Video Footage of Simulator

The following videos are provided to show the behavior of the different configurations.

[50 MPH Video](https://youtu.be/6Wr-LmXetTk)
[100 MPH Video](https://youtu.be/xepJJUM6dr4)



## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

### Parameters Tuning

The default parameters are tuned for high speeds. Positional parameters have been added to allow
for quick tuning for testing at different speeds. 

1. Steering actuator minimizer
2. Accelerator actuator minimizer
3. Steering smoothing
4. Acceleration smoothing
5. Target velocity 

# Reflection

The MPC was a fun and challenging project. Once the boiler plate code was in place, it was all 
about tuning the cost function to get desired results. Mostly, tuning weights to stabalize steering
under high speeds. The MPC proved to be a very powerful tool once a suggested line can be determined by path planning. 