# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./example.png "Project Example"

## Introduction
This is my implementation of the Path Planning project for the Udacity Self-Driving Car Nanodegree. This proved to be a challenging, but fun, project to implement.

## Discussion of rubric points

### Compilation
#### 1. The code compiles correctly.
The project was developed and tested on a Debian 9.1 (stretch) machine with Gcc 6.3.0. It compiles cleanly.

The original CMakeLists.txt was changed slightly because the implementation uses multiple .cpp files. All map-related code was moved to map.cpp/h. The bulk of the implementation is in planner.cpp/h, with a bit of vehicle abstraction code in vehicle.cpp/h.

### Valid trajectories
#### 1. The car is able to drive at least 4.32 miles without incident.
I've had multiple instances of the car drive 4+ laps around the track with incident. 

![alt text][image1]

The screenshot above comes from this exciting 30+ minute [YouTube Video](https://www.youtube.com/watch?v=Vki4USDME0E).

Occasionally the car cannot avoid an incident but often this an exceptional situation that could not be avoided without exceeding the maximum allowed acceleration or jerk by executing a emergency stop. An example of this would be a collision on the other side of the road and one of the cars getting catapulted right in front of the ego car. Unfortunately I don't have a screen capture.

These incidents happen so infrequently that is hard to develop and test a solution without a heavily improved version of the simulator.

#### 2. The car drives according to the speed limit.
When not obstructed by traffic the car tries to drive at 0.95 times the speed limit (47.5 mph). When the car gets to close to a leading vehicle in the same lane it changes speed to keep a safe distance.

#### 3. Max acceleration and jerk are not Exceeded.
Maximum acceleration is fixed at 5 m/s². The trajectory of the car is smoothed using a cubic spline and it should never exceed the maximum allowed jerk.

#### 4. Car does not have collisions.
The car slows down to maintain a safe distance to a leading car. When looking to change lanes the proposed trajectory is checked for possible collisions in the near future. Trajectories with possible collisions are not taken.

#### 5. The car stays in its lane, except for the time between changing lanes.
The car does its best to always drive in the center of the lanes (d=2, 6 or 10). The trajectory of a lane change should never take more than 3 seconds to complete.

However, there is (at least) one point on the track when the car is driving in the center of the right lane (d=10) but it is still reported as "Outside of Lane". I tried multiple ways of interpolating the waypoints but they all lead to the same problem. The YouTube video linked below is an example of this situation. Visually I can not detect the car leaving the lane...

[![Outside of Lane](https://img.youtube.com/vi/4DbbwSiFpGk/0.jpg)](https://www.youtube.com/watch?v=4DbbwSiFpGk)

#### 6. The car is able to change lanes.
The car tries to drive in the fastest lane possible. It checks upcoming traffic (for about 40m) and chooses the most desirable lane: if possible one without a car in that segment, else the one with the fastest moving car. If possible a lane change operation is then executed towards that lane.

Most of the time this algorithm allows the car to avoid being slowed down by traffic up ahead and change lanes without losing too much speed.

### Reflection

#### 1. There is a reflection on how to generate paths.
The code for the path generations can be found in planner.cpp and planner.h.

The car sets two targets waypoints ahead of the car. When a target waypoints is passed it's removed from the list and a new one further on the track is added to the list. A spline is fitted to the last passed waypoint, the ego car, and the two future waypoints. This results in a smooth path.

The planner always gives 50 points to the simulator and reuses the previous point given back by the simulator to ensure the temporal smoothness of the path.

The future position of the detected vehicles on the road is predicted every timestep of the generated path. The ego car tries to maintain a safe distance between the leading car in its lane.

Every time a target waypoint is reached the car also checks if it is driving in the best lane. If not it tries to change to that lane, taking a 5 second cooldown between lane changes in account. To execute a lane change new target waypoints are created with the d-value of the desired lane. This path is checked for collision with any other vehicle for the next 5 seconds. Only when no potential collision are detected is the lane change executed.

A simple state machine is used to control the behavior of the car. It has three states:
- startup: no lane changes are allowed when startup up the simulation to allow the car to reach a decent velocity
- keep lane : keep driving in the same lane and check regularly if a lane change is desirable.
- changing lane : during lane changes

### Future work
Although I feel the current implementation is sufficient to pass, I can see several potential improvement.
1. The "density" of the s coordinate doesn't seem the same along the entire track, especially in the far corners of the track. This leads to the car not driving at a constant velocity. Switching back to cartesian coordinates sooner could improve this but this leads to more complexity in the rest of the implementation (ie. collision detection).
2. The decision functions of the FSM are just some conditional logic. Changing to properly tuned cost functions would allow for better situational awareness.
3. The car drives around the track somewhat like a jerk. It stays in the left or center lanes when there is no one in the right lane and passes other cars on the right side at full speed. The FSM should be improved to minimize this behavior. But in the car's defense, the other cars in the simulator exhibit the same driving style with no apparent remorse.


---
Original README.md follows below:

---
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

