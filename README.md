# LQR-4-DOF-Manipulator
A control and simulation framework on MATLAB, implementing LQR control and CVX Trajectory smoothening for a 4-DOG Planar Robotic Manipulator


# 4-DOF Planar Manipulator: Inverse Kinematics and Control

This repository contains a MATLAB implementation of inverse kinematics, trajectory planning, and optimal control for a 4-DOF planar robotic manipulator.

## Overview

This project implements a comprehensive control framework for a 4-DOF planar manipulator, with emphasis on:

- Robust inverse kinematics solution
- Trajectory generation with optimization-based smoothing
- Linear Quadratic Regulator (LQR) controller design
- Performance analysis and visualization

The system can track arbitrary waypoint-based trajectories while minimizing acceleration, jerk, and control effort.

<img width="743" alt="Screenshot 2025-05-10 at 2 22 15 PM" src="https://github.com/user-attachments/assets/223e1f9b-1878-4837-b21b-c246ecba039b" />


## Repository Structure

- `kinematics.m` - Forward kinematics and Jacobian computation
- `inverse_kinematics.m` - Inverse kinematics solver with multiple fallback strategies
- `dynamics.m` - Linearized state-space model of the manipulator
- `lqr_controller.m` - LQR controller design
- `trajectory_gen.m` - Trajectory generation and smoothing
- `robot_sim.m` - Simulation of the controlled robot
- `analysis.m` - Performance metrics calculation and visualization
- `visualization.m` - Plotting and animation utilities
- `main.m` - Main execution script

## Features

### Inverse Kinematics
- Iterative Jacobian-based solver
- Multiple fallback methods for handling singularities
- Automatic handling of unreachable targets
- Multiple initial configurations for finding better solutions

### Trajectory Generation
- Waypoint-based planning in task space
- CVX-based trajectory smoothing 
- Optimization for reducing acceleration and jerk
- Comprehensive comparison between original and smoothed trajectories

### Control System
- LQR state feedback controller
- Linearized dynamics model
- Time-varying linearization during simulation
- Optional disturbance injection for robustness testing

### Analysis and Visualization
- Tracking error metrics (RMS, maximum, average)
- Control effort analysis with frequency spectrum
- Smoothing performance metrics (acceleration and jerk reduction)
- Robot animation with path tracing
- Workspace visualization

## Getting Started

### Prerequisites
- MATLAB (developed with R2024a)
-  CVX optimization toolbox for enhanced trajectory smoothing

### Installation
1. Clone this repository
2. Install CVX and ensure it's in your MATLAB path for enhanced trajectory smoothing

### Running the Simulation
1. Open MATLAB and navigate to the project directory
2. Run `main.m` to execute the full simulation pipeline
3. To modify the trajectory, edit the waypoints in `main.m`
4. To adjust controller parameters, modify the Q and R matrices in `main.m`

## Example Usage

```matlab
% Define task-space waypoints for a square path
waypoints = [
    2, 2, -2, -2, 2;   % x-coordinates
    1, 3, 3, 1, 1      % y-coordinates
];

% Generate trajectory through waypoints
[t_traj, joint_traj] = trajectory_gen(waypoints, 10, 1001);

% Design LQR controller
[A, B, C, D] = dynamics(joint_traj(:,1));
Q = diag([10, 10, 10, 10, 1, 1, 1, 1]); % State weights
R = diag([0.1, 0.1, 0.1, 0.1]);        % Control weights
[K, lqr_info] = lqr_controller(A, B, Q, R);

% Simulate the system
options.t_final = 10;
options.dt = 0.01;
options.disturbances = false;
[t, x, y, u] = robot_sim(trajectory, K, options);

% Analyze performance
metrics = analysis(t, x, y, u, trajectory);
```

## Results

The system demonstrates excellent tracking performance:
- RMS position error typically below 0.05m
- High reduction in acceleration (40-50%) and jerk (70-80%) with smoothing
- Stable control with reasonable control effort

## Visualization Examples

- Joint trajectories (smoothed vs. original)
- Task-space path following
- Control effort and its frequency spectrum
- Tracking error analysis
- Robot animation

## Theory and Implementation

### State-Space Model
The manipulator is modeled as:
- State: x = [θ₁, θ₂, θ₃, θ₄, θ̇₁, θ̇₂, θ̇₃, θ̇₄]ᵀ
- Input: u = [τ₁, τ₂, τ₃, τ₄]ᵀ
- Output: y = [xₑₑ, yₑₑ, ẋₑₑ, ẏₑₑ]ᵀ

The system matrices are:
- A: Double integrator structure
- B: Maps torques to joint accelerations
- C: Relates joint states to end-effector position/velocity using the Jacobian
- D: Zero (no direct feedthrough)

### Inverse Kinematics
Uses an iterative approach based on Jacobian linearization:
1. Compute forward kinematics and Jacobian at current configuration
2. Calculate position error
3. Update joint angles using pseudoinverse or Jacobian transpose methods
4. Repeat until convergence

### Trajectory Smoothing
The smoothing optimization minimizes:
```
minimize: w₁‖q-q_orig‖² + w₂‖dq/dt‖² + w₃‖d²q/dt²‖² + w₄‖d³q/dt³‖²
```
subject to boundary conditions that preserve the start and end positions.

### LQR Control
The LQR controller minimizes:
```
J = ∫[x^T·Q·x + u^T·R·u]dt
```
By solving the continuous-time algebraic Riccati equation (CARE).

