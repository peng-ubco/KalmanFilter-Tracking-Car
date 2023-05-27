# Tracking an autonomous car using a Kalman filter

Track a 2D moving object is a common application for Kalman filter.

We use an autonomous car as an example to demonstrate some interesting points.

## Tracking 2D object without orientation

Simply run the "kalman_2Dtracking.py"

![alt text](https://github.com/peng-ubco/KalmanFilter-Tracking-Car/blob/main/kf_2d_tracking.gif | width=30)

You may have observed that the estimate accuracy degrades when the car is making a turn at the traffic light intersection. 

How to fix this problem? Adding the orientation information can help.

## Tracking 2D object with orientation 

Here, we make the situation even more challenging by considering the car is driving in circle. 
So that we can observe the effect of including the orientation infomation more clearly. 
In this case, the measurements are pos_x, pos_y, and current angle of the car.

Of course, the programe is a bit trikier. I will introduce two different methods to tackle this challenge.
The difference exists in how we construct the state vector.

## Method 1
State vector is constructed as [x, y, $\theta$, $\dot{x}$, $\dot{y}$, $\dot{\theta}$],
where the x is the position at x-axis, 
y is the position at y_axis, 
and $\theta$ is the orientation (angle) of the car, 
$\dot{x}$ is the velocity at x-axis, 
$\dot{y}$ is the velocity at y-axis,
and $\dot{\theta}$ is the velocity of the angle change

Before running the file, make sure the three parameters are set as "True", as shown below: 
```sim_opt['DRIVE_CIRCLE'] = True```
```sim_opt['MEASURE_ANGLE'] = True```
`sim_opt['CONTROL_INPUTS'] = True`  

![alt text](https://github.com/peng-ubco/KalmanFilter-Tracking-Car/blob/main/kf_2d_tracking_circle.gif | width=50)

## Method 2

State vector is constructed as [x, y, v, $\theta$, $\dot{\theta}$]
where v is the velocity of the car, and other notations are as described above.

Under development....







