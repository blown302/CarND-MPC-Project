# Model Predictive Control Project
Implementation of a MP Controller for the Self-Driving Car Engineer Nanodegree Program

---

# Strategy 
The goal of the project was to reach high speed. To achieve this goal, a combination of steering
minimizing, steering smoothing and velocity management was utilized. In addition Latency was
was accounted for by calculating the time delta and performing a calculation with a `Kitmatic Model`
to produce the starting point for the solver.

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