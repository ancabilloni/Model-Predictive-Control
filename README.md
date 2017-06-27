# Model Predictive Control
## Introduction
This project is to create a predictive model that controls self driving car staying in lanes from the environment around.

[Result Video](https://www.youtube.com/watch?v=N33DK5Sd-7M)

### Project Folder Structure
1. ./src/main.cpp: main code structure
2. ./src/MPC.cpp & MPC.h: Model Predictive Control calculation
3. Language: C++

## Implementation 

### Vehicle Models
The state for the model is __[x, y, psi, v, cte, epsi]__
with __[x,y,psi]__ are 2D coordinate and orientation of the vehicle. __v__ is velocity,
 __cte__ is cross track error and _epsi_ is orientation error. 

Actuator of the model is __[delta, a]__ with __delta__ is the steering angle and __a__ is the acceleration.
For simplification, accleration values range from [-1,1] with positive is accelerating and negative is braking.

### Cost Functions (in MPC.cpp)
I set up cost functions for __cte, epsi, velocity, steering angle, and acceleration__ as below:
```
    fg[0] = 0;
    // Cost based on the reference state
    for (int i = 0; i < N; i++){
      fg[0] += 2500*CppAD::pow(vars[cte_start + i], 2);
      fg[0] += 2500*CppAD::pow(vars[epsi_start + i], 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators
    for (int i = 0; i<N-1; i++){
      fg[0] += 5*CppAD::pow(vars[delta_start + i], 2);
      fg[0] += 5*CppAD::pow(vars[a_start + 1], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i<N-2; i++){
      fg[0] += 2000*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start+i], 2);
      fg[0] += CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
    }
```

### Constraints for MPC in (MPC.cpp)
The following equations are used to update the kinematic models
```
x_(t+1) = x_t + v_t * cos(psi_t) * dt
y_(t+1) = y_t + v_t * sin(psi_t) * dt
psi_(t+1) = psi_t + v_t * delta_t * dt / Lf
v_(t+1) = v_t + a_t * dt
cte_(t+1) = f(x_t) - y_t + (v_t * sin(epsi_t) * dt)
epsi_(t+1) = psi_t - desired_psi_t + (v_t/L_f) * delta_t * dt
```
With: 

`dt` is the timestep.
`Lf` is the distance between front wheels to center of gravity.
`f(x_t)` is the trajectory determine by polynomanial line of waypoints.
`desired_psi_t` is the arctan of `f(x_t)` slope.

 From the equations above, the constraints for the model can be set up as following:
 ```
 for (int i = 1; i < N; i++) {
      // State at time t+1
      AD<double> x1 = vars[x_start + i];
      AD<double> y1 = vars[y_start + i];
      AD<double> psi1 = vars[psi_start + i];
      AD<double> v1 = vars[v_start + i];
      AD<double> cte1 = vars[cte_start + i];
      AD<double> epsi1 = vars[epsi_start + i];

      // State at time t
      AD<double> x0 = vars[x_start + i - 1];
      AD<double> y0 = vars[y_start + i - 1];
      AD<double> v0 = vars[v_start + i - 1];
      AD<double> psi0 = vars[psi_start + i - 1];
      AD<double> cte0 = vars[cte_start + i - 1];
      AD<double> epsi0 = vars[epsi_start + i - 1];

      // Actuator at time t
      AD<double> delta0 = vars[delta_start + i - 1];
      AD<double> a0 = vars[a_start + i - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

      // Constraints
      fg[1 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + i] = psi1 - (psi0 - v0 * delta0 * dt / Lf);
      fg[1 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + i] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + i] = epsi1 - (psi0 - psides0 - v0 * delta0 * dt / Lf);
    }
 ```
 ### Numbers of Variables, Constraints and Boundaries Limit (in MPC.cpp)
 As already mentioned, the state has 6 elements, and the actuations has 2 elements. Hence, the number of variables and constraints can be defined as following:
 ```
 n_vars = N*6 + (N-1)*2
 n_constraints = N*6
 ```
 with N is the number of timesteps that can be manually defined.
 
 #### Boundary Limits
 The boundaries for all the elements are set as following:
 ```
   for (int i=0; i < delta_start; i++){
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  //Set [-25,25] limits for delta
  for (int i = delta_start; i < a_start; i++){
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Set [-1,1] limits for a
  for (int i = a_start; i < n_vars; i++){
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  ```
  ### Fitting Polynomials (in main.cpp)
  To simplify the values in coordinates and update equations, distance from vehicle to waypoints are taken into consideration. A transformation to have vehicle x coordinates completely to the right was also made. A 3rd order polynomial was chosen to fit the waypoints trajectory.
  
  ```
  Eigen::VectorXd x_transform(ptsx.size());
  Eigen::VectorXd y_transform(ptsy.size());
  for (int i=0; i < ptsx.size(); i++){
    x_transform[i] = (ptsx[i] - px)*cos(-psi) - (ptsy[i] - py)*sin(-psi);
    y_transform[i] = (ptsx[i] - px)*sin(-psi) + (ptsy[i] - py)*cos(-psi);
   }

   auto coeffs = polyfit(x_transform, y_transform, 3);
   double cte = polyeval(coeffs, 0);
   // double epsi = psi - atan(coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px);
   double epsi = -atan(coeffs[1]);

   Eigen::VectorXd state(6);
   state << 0, 0, 0, v, cte, epsi;
   ```
   ### Latency
   In real driving scenarios, there will be some delay for the actuation command to execute as the command propagates through the system. Hence, some delay of about 100 milliseconds is considered for this model for the purpose of mimic the real driving.
   
   However, as there is delay, that means the vehicle models should be udpated during this 100 ms as the car is still moving. To take care of this latency problem, some update in the state are applied as following:
   
   ```
// Predicting 100 ms in the future.
double latency = 0.1;
const double Lf = 2.67;
state[0] = v*cos(0)*latency;
state[1] = v*sin(0)*latency;
state[2] = (-v*steer_value*latency/Lf);
state[3] = v + throttle_value*latency;
state[4] = cte + v*sin(epsi)*latency;
state[5] = epsi - (v/Lf)*steer_value*latency;
```
### Choose N and dt values
This is purely trial and errors. I started with N = 10 and dt = 0.1, without latency and latency update, and the vehicle performs very well the entire track on simulator up to 60 mph. 

However, after adding 100 ms, the vehicle started to get unstable and did not make it through the track. From here, I added latency update equations, and the vehicle got back to more stable driving to perform the entire track. When I tried N = 10, dt = 0.15, the vehicle performs even more stable in addition to the latency update. This is why I choose N = 10 and dt = 0.15 as my final values.
         
  
  
 
 
 
