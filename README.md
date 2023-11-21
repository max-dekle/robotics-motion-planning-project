# Robotics Control Software - Project Overview
## Introduction
This document provides an overview of Mobile Manipulation Simulator, a capstone project for the Modern Robotics certificate from Northwestern University. This project focuses on developing advanced control algorithms for a pick-and-pace robotic arm. The modern_robotics library is utilized throughout to facilitate both kinematic and dynamic calculations. Key features include trajectory generation, feedback control, and handling of various robotic configurations and states.

The full program can be run using 'python main.py'. Choose one of 'Best', 'Overshoot', and 'newTask' to uncomment in the main() function to see how the arm performs under different parameters. 

## Key Components
- Trajectory Generation: Utilizing the modern_robotics library, we create smooth trajectories for the robot's end-effector to follow, ensuring precise and efficient movements.
- Feedback Control: Implementing robust feedback control mechanisms to correct any deviations from the desired trajectory.
- Jacobian Computation: Calculating the Jacobian matrix to relate joint velocities to end-effector velocities, allowing for effective motion planning.
- NextState: Update the configuration of the chassis as well as the wheel and arm angles. 

## Results and Observations
Initially, the robotic arm was oscillating a lot and not reliably picking up the cube. To address this, different Kp and Ki values were tried, with Kp = 30 and Ki = 0.75 found to be optimal. Low Kp and Ki values typically resulted in overshooting and high oscillation. On the other hand, high Ki values typically exacerbated existing errors in the trajectory generation. More information can be seen in the results folder.

## Future Work
- Adaptive Control: Implementing adaptive control strategies to enable the robot to adjust its behavior based on environmental changes.
- Machine Learning Integration: Exploring the integration of machine learning algorithms for predictive control and autonomous decision-making.
