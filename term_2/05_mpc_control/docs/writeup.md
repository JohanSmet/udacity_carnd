# Write Up
Answers to the questions in the project [rubric](https://review.udacity.com/#!/rubrics/896/view).

[image1]: ./equations.png "Motion Model update equations"

### 1. Student describes their model in detail. This includes the state, actuators and update equations.
My implementation uses the Global Kinematic Model as seen in the lectures. 

The state consists of :
- the position of the vehicle (x & y)
- the orientation of the vehicle (Ψ or psi)
- the velocity of the vehicle (v)
- the current cross track error (cte) 
- the current deviation from the desired orientation (eΨ or epsi)

The actuators are:
- throttle (a) to control the acceleration of the vehicle
- steering value (δ or delta) to control the orientation change of the vehicle

The following equations are used the compute the state of the vehicle model at the next timestep:

![alt text][image1]

### 2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
I started with N = 10 and dt = 0.1 mainly because predicting a second in the future in 10 steps seemed a reasonable start. 

I tried both more and larger timesteps (N = 20 and dt = 0.2 but also N = 20 and dt=0.1) but found that the predicted path was more unstable, especially in corners. I also tried N = 20 and dt=0.05 to reduce the predicted time back to 1 second but that did not yield sufficient advantages to offset the additional computational costs.

At last I settled on N = 10 and dt=0.75 because that seemed to react faster in corners while still being stable in a straight part of the course.

## 3. A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
The only preprocessing I applied to the waypoints was to transform them into the coordinate system of the car. I did this mainly because the resulting path has be sent back to the simulator cars' coordinate system.

The vehicle state was also updated to deal with the latency requirements of the project. See next question for details.

## 4. Student provides details on how they deal with latency.
To deal with the latency in applying the updated actuators I predict the vehicle state 100ms in the future using the motion model before passing the values to the MPC procedure. The code for this is in main.cpp at lines 118-126.
