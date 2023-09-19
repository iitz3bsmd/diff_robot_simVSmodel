# Differential Robot Simulation vs Model / diff_robot_simVSmodel

![diff_robot_simVSmodel](https://github.com/iitz3bsmd/diff_robot_simVSmodel/assets/112030326/de3f8b04-b855-4748-bd3d-10012a7f966b)

## Description
this project aims to compare two ways to estimate the position of a four-wheeled differential robot, first by using the mathematical model of the robot and feeding it to the Euler numerical method integrator, and the other ways was through simulating the robot commands on a simulation software.
The tools used: python, ros2, coppeliasim (V-REP) 

## Demo


## Launching
1. Open the `robot.ttt` scene in coppeliasim
2. open the ros2 node using the follwing command:
```
ros2 run simVSmodel comparator
```
