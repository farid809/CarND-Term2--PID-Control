# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program


## CarND Term2 :: Project 4 -- PID Controller Project

In this project, we are implementing a PID controller in C++ to maneuver a vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.



## Effect each of the P, I, D components On the Process

### Proportional Controller only : 
Increasing the proportional gain Kp will result in increasing the speed of the control system response (rise time) and reducing steady-state error (CTE In this case). however, push Kp to higher values will cause the system to be oscillatory.

### Effect of Derivative Gain:
Increasing the derivative gain will cause the system to react strongly to the rate of change in error dError/dTime. Hence Kd components help with eliminating oscillation. 

### Effect of Integral Gain: 
Integral gain reacts strongly for the cumulative error over time. and hence help with reducing steady state Error.



##Tuning the PID Controller 
Due to the lack of mathematical system dynamic model that describes the process in terms of differential equations and transfer functions I  resorted to online PID controller tuning methods below. It's also possible to online tune the PID Controller using simulation software such as Matlab Control toolbox by identifying the plant model using sample input/output.


### Tuning Using Ziegler–Nichols Method:
Initially, I attempted to tune the controller using Zeigler Nichols heuristic method. I implemented the below code in main.cpp to detect and calculate the period of oscillation. Then followed ZN method steps below to calculate the PID gain:

1. set Kd and Ki to zero.
2. Increase Kp until the process reaches the ultimate Gain Ku at which system will stable and consistent oscillation with period Tu.
3. Calculate the PID gains using the below formulas. 
    kp=Ku/5
    Ki=0.66*Ku/Tu
    Kd=2*Ku*Tu/3

However, I couldn't obtain working PID parameters using this method so I resorted to Manual Tuning method


### Manual Tuning
Here are the steps I followed during manual tuning.

1. started with the Kp gain calculated using the ZN method.
2. set Ki and Kd to Zero (0)
3. Increase Kp gain until system become oscillatory
4. Increase Kd gain until oscillation goes away
5. repeated 2 and 3 until increasing Kd gain doesn't stop oscillation

At this point, I was able to obtain PD values that good enough so the car can complete the track and no tire leaving the drivable portion of the track surface. however, I continued with Ki tuning

6. Decrease Kp by a factor of 2
7. set Ki to about 1% of Kp
8. increase Ki until oscillation starts again
9. decrease Ki by a factor of 2




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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

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

