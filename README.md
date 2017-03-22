yarp-wholebody-motorcontrol-example
===================================

Example of whole body torque control of a robot exposing YARP interface for motor control.

Compilation
-----------

[Install YARP](http://www.yarp.it/install.html). 

Then, clone this repo and build it using CMake (run this command from this repo directory): 
~~~
mkdir build
cd build
ccmake ..
cmake --build . 
~~~

Run
---
To run the example as it is, launch a iCub simulator (either `iCub_SIM` provided in the `icub-main` repository
 or the [Gazebo-based iCub's simulation](https://github.com/robotology/icub-gazebo)) and then launch the compile program:
~~~
./yarpWholeBodyMotorControlExample
~~~

or `yarpWholeBodyMotorControlExample.exe` on Windows. 

To see that the robot is actually switching control mode, launch the `yarpmotorgui` on the `left_leg` tab
to see that the control mode of the joints is changing when the `yarpWholeBodyMotorControlExample` is launched.