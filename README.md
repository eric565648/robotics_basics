# Robotics Basics

This repo utilized python [general_robotics_toolbox](https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py) developed in RPI to perform
- Forward/Inverse kinematics
- Iterative Jacobian inverse kinematics
- Path planning with Jacobian matrix
- Much more to come

The results are demonstrated in RobotStudio, ABB Robot simulator. This repo also demonstrats how to run the planning path on a robot using [abb_motion_program_exec](https://github.com/rpiRobotics/abb_motion_program_exec), a python-ABB interface developed in RPI

The repo is dedicated to show how the robotics math is realized with codes.

## Pre-request
- python >= 3.8
- numpy
- general-robotics-toolbox
- qpsolvers

### Install general-robotics-toolbox
```
pip install general-robotics-toolbox
```

### Install abb_motion_program_exec

See [here](https://github.com/rpiRobotics/abb_motion_program_exec) to install the library.

### Install qpsolvers with "quadprog" solver
```
pip install qpsolvers[quadprog]
```
