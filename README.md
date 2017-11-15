# PID controllers
## Udacity Self-Driving Car Engineer Nanodegree Program

This repository contains my implementation of a PID controller for the Udacity Self-Driving Car Engineer Nanodegree Program. The project uses the Udacity starter code as a basis, only PID.cpp and the corresponding header file have been altered by me.

## Reflection

### 1. Describe the effect each of the P, I, D components had in your implementation.

The **'P'roportional** component sets the steering value to a multiple of the current error by multiplying with a constant gain Kp. This means that the further the car is from the desired trajectory the harder it will steer towards the desired trajectory. When the gain is too low it will take long to reach the desired trajectory, when the gain is to high the car will have a tendency to overshoot and oscillate between the two sides of the desired trajectory.

An example of the behavior of a P-controller can be seen in the next video on YouTube (click the image):

[![P-controller](https://img.youtube.com/vi/g6x-cP5WO8o/0.jpg)](https://www.youtube.com/watch?v=g6x-cP5WO8o)

The **'D'erivative** component takes the slope of the change in error-value into account. This will cause the car to steer less as the error decreasing reducing the overshooting and oscillation as encountered with a simple P-controller.

The **'I'ntegral** component uses the surface area between the path of the car and the desired trajectory. This is used to correct noise or errors (e.g. misaligned wheels). When the cars follows the desired trajectory pretty closely the area between the car's trajectory and above and below the desired trajectory will cancel each other out. When there's an deviation for a while the area on one side will keep getting bigger until the I-component kicks in and starts correcting the error.

This is a video of the completely tuned PID-controller completing a lap in the simulator (click the image):
[![Full lap](https://img.youtube.com/vi/9kLyZjNEFlc/0.jpg)](https://www.youtube.com/watch?v=9kLyZjNEFlc)

### 2. Describe how the final hyperparameters were chosen.
I started by implementing the PID controller with the hyperparameters used in the lecture. When the PID controller was functioning as desired I added a twiddle routine to find more optimal hyperparameters. 

At first is set them all to 1 to let the twiddle-algorithm do its thing. But the car crashed rather quickly and this made this procedure rather tedious. I went back to my original hyperparameters (same as in the lecture). The car made it around the track with these and I used the twiddle-algorithm to optimize them.

In the beginning I let each cycle of the twiddle-algorithm run for 100 cycles but I found this to optimize for certain parts of the track and made performance on other parts worse. I increased the number of cycles to 2000 to take in account the majority of the track. Also I disregarded the first 100 samples to eliminate interference of the previous hyperparameters in the new measurements. I let this run for about two hours to arrive at the current hyperparameters:
- Kp = 0.226939
- Ki = 0.00397376
- Kd = 3.3


---
Original README.md follows below:

---



# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

