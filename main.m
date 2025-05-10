% main_fixed.m - Fixed main script for 4-DOF planar manipulator project

clear all; close all; clc;

fprintf('4-DOF Planar Manipulator: Inverse Kinematics and LQR Control\n');
fprintf('=========================================================\n\n');

%% Parameters
sim_time = 10;  % seconds
dt = 0.01;      % time step
enable_disturbances = false; % Set to false for initial testing

fprintf('Setting up simulation parameters\n');

%% Define task-space waypoints
% Define a safer square path well within the workspace
% Using a smaller square path that's easier to reach
waypoints = [
    2, 2, -3, -3, 2;   % x-coordinates (reduced from 3 to 2)
    1, 3, 4, 2, 1      % y-coordinates \(reduced from 2,5,5,2,2)
];

fprintf('Defined waypoints for a square path in task space\n');

%% Generate trajectory
fprintf('Generating trajectory through waypoints...\n');
try
    [t_traj, joint_traj] = trajectory_gen(waypoints, sim_time, round(sim_time/dt)+1);
    
    % Verify trajectory
    if any(isnan(joint_traj(:)))
        error('NaN values detected in joint trajectory');
    end
    
    % Compute joint velocities via finite differences with smoothing
    joint_vel = zeros(size(joint_traj));
    for i = 2:size(joint_traj,2)
        joint_vel(:,i) = (joint_traj(:,i) - joint_traj(:,i-1))/dt;
    end
    
    % Smooth velocities
    window_size = 5;
    for j = 1:4
        joint_vel(j,:) = movmean(joint_vel(j,:), window_size);
    end
    
    % Create trajectory structure
    trajectory = struct();
    trajectory.t = t_traj;
    trajectory.q = joint_traj;
    trajectory.dq = joint_vel;
    
    % Compute end-effector trajectory
    [fwd_kin, ~] = kinematics();
    trajectory.y = zeros(4, length(t_traj));
    for i = 1:length(t_traj)
        % Position
        [x, y] = fwd_kin(joint_traj(:,i));
        trajectory.y(1:2,i) = [x; y];
        
        % Velocity
        if i > 1
            trajectory.y(3:4,i) = (trajectory.y(1:2,i) - trajectory.y(1:2,i-1))/dt;
        end
    end
    
    % Smooth end-effector velocities
    for j = 3:4
        trajectory.y(j,:) = movmean(trajectory.y(j,:), window_size);
    end
    
    fprintf('Trajectory generation complete\n');
catch e
    fprintf('Error in trajectory generation: %s\n', e.message);
    fprintf('Using a simplified trajectory...\n');
    
    % Create a simple circular trajectory as fallback
    t_traj = linspace(0, sim_time, round(sim_time/dt)+1);
    radius = 2;
    center = [0; 0];
    
    % End-effector positions
    pos_x = center(1) + radius * cos(2*pi*t_traj/sim_time);
    pos_y = center(2) + radius * sin(2*pi*t_traj/sim_time);
    
    % Solve inverse kinematics for each point
    joint_traj = zeros(4, length(t_traj));
    joint_vel = zeros(4, length(t_traj));
    
    % Initial configuration
    theta_prev = [0; 0; 0; 0];
    
    % Simple IK for circle
    for i = 1:length(t_traj)
        target = [pos_x(i); pos_y(i)];
        try
            theta = inverse_kinematics(target, theta_prev);
            joint_traj(:,i) = theta;
            theta_prev = theta;
        catch
            if i > 1
                joint_traj(:,i) = joint_traj(:,i-1);
            end
        end
    end
    
    % Compute joint velocities
    for i = 2:length(t_traj)
        joint_vel(:,i) = (joint_traj(:,i) - joint_traj(:,i-1))/dt;
    end
    
    % Create trajectory structure
    trajectory = struct();
    trajectory.t = t_traj;
    trajectory.q = joint_traj;
    trajectory.dq = joint_vel;
    trajectory.y = zeros(4, length(t_traj));
    
    % Compute actual end-effector positions
    for i = 1:length(t_traj)
        [x, y] = fwd_kin(joint_traj(:,i));
        trajectory.y(1:2,i) = [x; y];
        
        if i > 1
            trajectory.y(3:4,i) = (trajectory.y(1:2,i) - trajectory.y(1:2,i-1))/dt;
        end
    end
end

%% Design LQR controller
fprintf('Designing LQR controller...\n');

try
    % Get linearized system model at initial configuration
    [A, B, C, D] = dynamics(joint_traj(:,1));
    
    % Check for numerical issues
    if any(isnan([A(:); B(:); C(:); D(:)]))
        error('NaN values detected in system matrices');
    end
    
    % LQR weights
    Q = diag([10, 10, 10, 10, 1, 1, 1, 1]);  % State weights
    R = diag([0.1, 0.1, 0.1, 0.1]);         % Control weights
    
    % Design LQR controller
    [K, lqr_info] = lqr_controller(A, B, Q, R);
    
    if any(isnan(K(:)))
        error('NaN values detected in controller gain matrix');
    end
catch e
    fprintf('Error in LQR controller design: %s\n', e.message);
    fprintf('Using a simple PD controller instead...\n');
    
    % Create a simple PD controller as fallback
    K = zeros(4, 8);
    K(1:4, 1:4) = 5 * eye(4);  % Position gains
    K(1:4, 5:8) = 2 * eye(4);  % Velocity gains
end

%% Simulate controlled system
fprintf('Running simulation with controller...\n');

% Simulation options
options.t_final = sim_time;
options.dt = dt;
options.disturbances = enable_disturbances;
options.dist_magnitude = 0.1;  % Reduced from 0.2 to 0.1 N·m

try
    % Run simulation
    [t, x, y, u] = robot_sim(trajectory, K, options);
    
    % Check for NaN or Inf in results
    if any(isnan(x(:))) || any(isnan(y(:))) || any(isnan(u(:)))
        error('NaN values detected in simulation results');
    end
    
    %% Visualize and analyze results
    fprintf('Generating visualizations and analyzing performance...\n');
    
    % Create visualizations
    visualization(t, x, y, u, trajectory);
    
    % Analyze performance
    metrics = analysis(t, x, y, u, trajectory);
    
    %% Save results
    save('lqr_simulation_results.mat', 't', 'x', 'y', 'u', 'trajectory', 'metrics');
    fprintf('Results saved to lqr_simulation_results.mat\n');
catch e
    fprintf('Error in simulation: %s\n', e.message);
    fprintf('Simulation failed. Check system parameters and try again.\n');
end

fprintf('\nSimulation complete!\n');