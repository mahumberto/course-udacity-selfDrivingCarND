# Project: Model Predictive Control

## 1. Goal
The goal of this project was to develop a Model Predictive Controller for the steering angle and throtle based on target trajectory input and perfect vehicle localization / error to the trajectory data. In addition, a controller output to actuator delay (latency) of 100ms was also simulated.

## 2. Controller
### 2.1. Inputs - Vehicle state 
The simulator environment provided several vehicle state inputs for the controller. See DATA.md for details on the data structure.
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.
* `psi` (float) - The orientation of the vehicle in **radians**

### 2.2. Inputs - Target trajectory and speed
* `ptsx` (Array<float>) - Target x trajectory (waypoints) given on global coordinates.
* `ptsy` (Array<float>) - Target y trajectory (waypoints) given on global coordinates.
* Internal controller parameter for target speed.

### 2.3. Outputs
* `steering_angle` (float) - Command for the steering angle in **radians**.
* `throttle` (float) - Command for the throttle and brake in a single variable (positive == gas // negative == brake).

### 2.4. Controller parameters
The controller selected the best fitting output to match the given target trajectory up until a parameter time window in the future. The time window was configured by 2 parameters: Number of steps (N) and time interval (dt). A compromise was required, since a Number of steps bigger to the selected (10) lead to a longer response time and the controller did not produce the outputs in time. This produced an unstable controller. After setting the N, the dt would determine how far into the future, the controller would try and match the reference trajectory. Another compromise was required here, since a a window too big would not correspond to the actual trajectory. In this case, a windos of dt=0.1 was selected, leading to a time frame of 1sec.    

## 3. Pipeline
The simulator provided the x,y pair of waypoints based on the simulator global position, but the Controller required the reference trajectory on vehicle based position. Therefore, the first preprocessing phase involved the transformation from the waypoints in two steps:
  * Move from the global position base to the vehicle coordinate base. 
  * Project the points in the vehicles x,y axes, which were rotated based on the vehicle orientation psi.

Following this coordinates conversion, the given set of target trajectory points was fed to a 3rd degree polyfit. This polyline was used finally used as target trajectory by the controller.  

The simulator added also a latency controller output to actuator action of 100ms which needed to be addressed. To overcome this latency, instead of feading the controller with the current vehicle state and related cte and epsi, a prediction from the vehicle states on the following 100ms was computed and this predicted state was then used as controller input. 

## 4. Build instructions
Please refer to build-instructions.md
