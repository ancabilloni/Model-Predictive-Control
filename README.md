# Model Predictive Control
## Introduction
This project is to create a predictive model that controls self driving car staying in lanes from the environment around.
Main language is C++.

### Project Folder Structure
1. ./src/Eigen-3.3: Eigen library for Vector and Matrix
2. ./src/json.hpp: 
3. ./src/main.cpp: main code structure
4. ./src/MPC.cpp & MPC.h: Model Predictive Control calculation

## Approach in MPC.cpp
The state for the model is [x, y, psi, v, cte, epsi]
with [x,y,psi] is 2D coordinate and orientation of the vehicle. [v] is velocity,
 [cte] is cross track error and `epsi` is orientation error. 

Actuator of the model is [delta, a] with `delta` is the steering angle and `a` is the acceleration.
For simplification, `a` values range from [-1,1] with positive is accelerating and negative is braking.

### Kinetic Model
x_(t+1) = x_t + v_t * cos(psi_t) * dt

y_(t+1) = y_t + v_t * sin(psi_t) * dt

psi_(t+1) = psi_t + v_t * delta_t * dt / Lf

v_(t+1) = v_t + a_t * dt

With: 

`dt` is the timestep.

`Lf` is the distance between front wheels to center of gravity.
 # Model-Predictive-Control
