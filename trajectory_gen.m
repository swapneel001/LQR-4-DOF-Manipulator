function [times, joint_trajectory] = trajectory_gen(waypoints, total_time, num_points)
    % Generate trajectory through waypoints in task space with CVX-based smoothing
    
    % Load kinematics functions
    [fwd_kin, ~] = kinematics();
    
    % Create time vector
    times = linspace(0, total_time, num_points);
    dt = times(2) - times(1);
    
    % Initialize output
    joint_trajectory = zeros(4, num_points);
    
    % Number of waypoints
    num_waypoints = size(waypoints, 2);
    
    % Waypoint times (evenly spaced)
    waypoint_times = linspace(0, total_time, num_waypoints);
    
    % Solve inverse kinematics for each waypoint
    waypoint_joints = zeros(4, num_waypoints);
    
    % Start with an upright configuration
    theta_prev = [0; 0; 0; 0];  
    
    % Generate better first guess for first waypoint
    target = waypoints(:,1);
    target_dist = norm(target);
    theta_prev = [atan2(target(2), target(1)); pi/4; -pi/4; 0];
    
    % Try to solve IK for each waypoint
    successful_waypoints = true;
    for i = 1:num_waypoints
        try
            % Use previous solution as initial guess
            waypoint_joints(:,i) = inverse_kinematics(waypoints(:,i), theta_prev);
            theta_prev = waypoint_joints(:,i);
            
            % Print progress
            fprintf('Solved IK for waypoint %d: (%.2f, %.2f)\n', ...
                    i, waypoints(1,i), waypoints(2,i));
            
            % Verify solution
            [x, y] = fwd_kin(waypoint_joints(:,i));
            error = norm([x; y] - waypoints(:,i));
            
            % If error is too large, try with different initial guesses
            if error > 0.1
                fprintf('Warning: Large error (%.4f) for waypoint %d. Retrying...\n', error, i);
                
                % Try with different initial configurations
                best_error = error;
                best_theta = waypoint_joints(:,i);
                
                % Configuration seeds
                initial_guesses = [
                    [0; 0; 0; 0],                                    % All zeros
                    [atan2(waypoints(2,i), waypoints(1,i)); 0; 0; 0], % First link toward target
                    [atan2(waypoints(2,i), waypoints(1,i)); pi/4; -pi/4; 0], % Elbow configuration
                    [-pi/4; pi/2; 0; 0],                             % Elbow up
                    [pi/4; -pi/2; 0; 0]                              % Elbow down
                ];
                
                for j = 1:size(initial_guesses, 2)
                    try
                        new_sol = inverse_kinematics(waypoints(:,i), initial_guesses(:,j));
                        [x_new, y_new] = fwd_kin(new_sol);
                        new_error = norm([x_new; y_new] - waypoints(:,i));
                        
                        if new_error < best_error
                            best_theta = new_sol;
                            best_error = new_error;
                            fprintf('Found better solution with error %.4f\n', best_error);
                        end
                    catch
                        fprintf('Initial guess %d failed for waypoint %d\n', j, i);
                    end
                end
                
                waypoint_joints(:,i) = best_theta;
                error = best_error;
            end
            
            % Final check if waypoint is acceptable
            if error > 0.5 || any(isnan(waypoint_joints(:,i)))
                fprintf('Warning: Waypoint %d has poor solution (error=%.4f)\n', i, error);
                successful_waypoints = false;
            end
        catch e
            fprintf('Error solving IK for waypoint %d: %s\n', i, e.message);
            successful_waypoints = false;
            % Use previous joint solution as fallback
            if i > 1
                waypoint_joints(:,i) = waypoint_joints(:,i-1);
            else
                waypoint_joints(:,i) = [0; 0; 0; 0];
            end
        end
    end
    
    % Generate initial trajectory
    if successful_waypoints && num_waypoints >= 2
        fprintf('All waypoints successfully solved, generating initial trajectory\n');
        
        % For each joint, create cubic spline through waypoints
        for j = 1:4
            joint_trajectory(j,:) = interp1(waypoint_times, waypoint_joints(j,:), ...
                                          times, 'spline');
        end
    else
        fprintf('Some waypoints failed, using simplified trajectory\n');
        
        % Use linear interpolation between valid waypoints
        valid_indices = find(~any(isnan(waypoint_joints), 1));
        if length(valid_indices) >= 2
            fprintf('Using %d valid waypoints for trajectory\n', length(valid_indices));
            valid_waypoint_times = waypoint_times(valid_indices);
            valid_joint_angles = waypoint_joints(:, valid_indices);
            
            % Linear interpolation for trajectory
            for j = 1:4
                joint_trajectory(j,:) = interp1(valid_waypoint_times, valid_joint_angles(j,:), ...
                                             times, 'linear', 'extrap');
            end
        else
            fprintf('Not enough valid waypoints, using constant trajectory\n');
            % Use a safe configuration
            for i = 1:num_points
                joint_trajectory(:,i) = [0; 0; 0; 0];
            end
        end
    end
    
    % Store the original unsmoothed trajectory for later comparison
    original_joint_trajectory = joint_trajectory;
    
    % Apply CVX-based smoothing

    fprintf('CVX trajectory smoothing\n');
    joint_trajectory = smooth_trajectory_cvx(joint_trajectory, dt);


    
    % Save original trajectory for analysis
    original_traj = struct();
    original_traj.t = times;
    original_traj.q = original_joint_trajectory;
    
    % Calculate joint velocities for the original trajectory
    original_traj.dq = zeros(size(original_joint_trajectory));
    for i = 2:size(original_joint_trajectory,2)
        original_traj.dq(:,i) = (original_joint_trajectory(:,i) - original_joint_trajectory(:,i-1))/dt;
    end
    
    % Compute end-effector positions and velocities for original trajectory
    original_traj.y = zeros(4, num_points);
    for i = 1:num_points
        [x, y] = fwd_kin(original_joint_trajectory(:,i));
        original_traj.y(1:2,i) = [x; y];
        
        if i > 1
            original_traj.y(3:4,i) = (original_traj.y(1:2,i) - original_traj.y(1:2,i-1))/dt;
        end
    end
    
    % Save original trajectory
    save('original_trajectory.mat', 'original_traj');
    
    % Verify trajectory meets task-space requirements
    task_trajectory = zeros(2, num_points);
    for i = 1:num_points
        [x, y] = fwd_kin(joint_trajectory(:,i));
        task_trajectory(:,i) = [x; y];
    end
    
    % Visualize the trajectories (both original and smoothed)
    visualize_trajectories(times, joint_trajectory, task_trajectory, waypoints, original_joint_trajectory);
    
    fprintf('Trajectory generation complete with smoothing\n');
end

function smoothed_trajectory = smooth_trajectory_cvx(original_trajectory, dt, weights)
    % Smooth the trajectory using CVX optimization
    % original_trajectory: original joint angles over time [4 x num_points]
    % dt: time step between points
    % weights: structure with weights for different optimization terms
    
    % Default weights if not provided
    if nargin < 3
        weights = struct();
        weights.position = 1.0;       % Weight for trajectory following
        weights.velocity = 0.1;       % Weight for velocity smoothness
        weights.acceleration = 0.01;  % Weight for acceleration smoothness
        weights.jerk = 0.001;         % Weight for jerk minimization
    end
    
    [num_joints, num_points] = size(original_trajectory);
    
    % Initialize the smoothed trajectory
    smoothed_trajectory = zeros(size(original_trajectory));
    
    % Precompute finite difference matrices for derivatives
    % First derivative (velocity)
    D1 = spdiags([-ones(num_points,1) ones(num_points,1)], [0 1], num_points-1, num_points) / dt;
    
    % Second derivative (acceleration)
    D2 = spdiags([ones(num_points,1) -2*ones(num_points,1) ones(num_points,1)], [0 1 2], num_points-2, num_points) / dt^2;
    
    % Third derivative (jerk)
    D3 = spdiags([-ones(num_points,1) 3*ones(num_points,1) -3*ones(num_points,1) ones(num_points,1)], [0 1 2 3], num_points-3, num_points) / dt^3;
    
    % Process each joint independently
    for joint = 1:num_joints
        % Original trajectory for this joint
        orig = original_trajectory(joint, :)';
        
        % Set up the optimization problem
        cvx_begin 
            variable smoothed(num_points)
            
            % Objective: minimize weighted sum of terms
            obj = weights.position * norm(smoothed - orig, 2) + ...
                  weights.velocity * norm(D1*smoothed, 2) + ...
                  weights.acceleration * norm(D2*smoothed, 2) + ...
                  weights.jerk * norm(D3*smoothed, 2);
            minimize(obj)
            
            % Constraints: keep start and end points fixed
            subject to
                smoothed(1) == orig(1);
                smoothed(num_points) == orig(num_points);
            
        cvx_end
        
        % Store the result
        smoothed_trajectory(joint, :) = smoothed';
        fprintf('CVX smoothing completed for joint %d\n', joint);
    end
    
    fprintf('Trajectory smoothing with CVX complete\n');
end

function visualize_trajectories(times, joint_trajectory, task_trajectory, waypoints, original_joint_trajectory)
    % Create plots to visualize both original and smoothed trajectories
    figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 1200, 800]);
    
    % Joint trajectory comparison
    subplot(2,2,1);
    plot(times, joint_trajectory', 'LineWidth', 1.5);
    title('Smoothed Joint Trajectory');
    xlabel('Time (s)');
    ylabel('Joint Angle (rad)');
    legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
    grid on;
    
    subplot(2,2,2);
    plot(times, original_joint_trajectory', 'LineWidth', 1.5);
    title('Original Joint Trajectory');
    xlabel('Time (s)');
    ylabel('Joint Angle (rad)');
    legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
    grid on;
    
    % Task-space trajectory
    subplot(2,2,3);
    plot(task_trajectory(1,:), task_trajectory(2,:), 'b-', 'LineWidth', 2);
    hold on;
    
    % Get original end-effector trajectory
    [fwd_kin, ~] = kinematics();
    original_task_trajectory = zeros(2, size(original_joint_trajectory, 2));
    for i = 1:size(original_joint_trajectory, 2)
        [x, y] = fwd_kin(original_joint_trajectory(:,i));
        original_task_trajectory(:,i) = [x; y];
    end
    
    plot(original_task_trajectory(1,:), original_task_trajectory(2,:), 'r--', 'LineWidth', 1.5);
    plot(waypoints(1,:), waypoints(2,:), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    title('End-Effector Trajectories');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend('Smoothed', 'Original', 'Waypoints');
    grid on;
    axis equal;
    
    % Compute and plot trajectory derivatives
    subplot(2,2,4);
    
    % Calculate velocities, accelerations, and jerks
    dt = times(2) - times(1);
    
    % Joint velocities
    smoothed_vel = zeros(4, length(times)-1);
    original_vel = zeros(4, length(times)-1);
    
    % Joint accelerations
    smoothed_accel = zeros(4, length(times)-2);
    original_accel = zeros(4, length(times)-2);
    
    % Joint jerks
    smoothed_jerk = zeros(4, length(times)-3);
    original_jerk = zeros(4, length(times)-3);
    
    % Compute derivatives
    for i = 1:4 % For each joint
        % Velocity
        smoothed_vel(i,:) = diff(joint_trajectory(i,:))/dt;
        original_vel(i,:) = diff(original_joint_trajectory(i,:))/dt;
        
        % Acceleration
        smoothed_accel(i,:) = diff(smoothed_vel(i,:))/dt;
        original_accel(i,:) = diff(original_vel(i,:))/dt;
        
        % Jerk
        smoothed_jerk(i,:) = diff(smoothed_accel(i,:))/dt;
        original_jerk(i,:) = diff(original_accel(i,:))/dt;
    end
    
    % Compute norms of derivatives
    smoothed_vel_norm = vecnorm(smoothed_vel);
    original_vel_norm = vecnorm(original_vel);
    smoothed_accel_norm = vecnorm(smoothed_accel);
    original_accel_norm = vecnorm(original_accel);
    smoothed_jerk_norm = vecnorm(smoothed_jerk);
    original_jerk_norm = vecnorm(original_jerk);
    
    % Plot acceleration comparison
    t_accel = times(1:end-2);
    
    plot(t_accel, original_accel_norm, 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t_accel, smoothed_accel_norm, 'b-', 'LineWidth', 1.5);
    
    title('Joint Acceleration Magnitude');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s²)');
    legend('Original', 'Smoothed');
    grid on;
    
    % Calculate smoothing metrics for display
    accel_reduction = (mean(original_accel_norm) - mean(smoothed_accel_norm)) / mean(original_accel_norm) * 100;
    jerk_reduction = (mean(original_jerk_norm) - mean(smoothed_jerk_norm)) / mean(original_jerk_norm) * 100;
    
    fprintf('\nTrajectory Smoothing Metrics:\n');
    fprintf('  Acceleration reduction: %.2f%%\n', accel_reduction);
    fprintf('  Jerk reduction: %.2f%%\n', jerk_reduction);
    
    % Additional figure for detailed derivative analysis
    figure('Name', 'Derivative Analysis', 'Position', [150, 150, 1200, 600]);
    
    % Velocity plot
    subplot(1,3,1);
    t_vel = times(1:end-1);
    plot(t_vel, original_vel_norm, 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t_vel, smoothed_vel_norm, 'b-', 'LineWidth', 1.5);
    title('Joint Velocity Magnitude');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend('Original', 'Smoothed');
    grid on;
    
    % Acceleration plot (already shown in previous figure, but included for completeness)
    subplot(1,3,2);
    plot(t_accel, original_accel_norm, 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t_accel, smoothed_accel_norm, 'b-', 'LineWidth', 1.5);
    title('Joint Acceleration Magnitude');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s²)');
    legend('Original', 'Smoothed');
    grid on;
    
    % Jerk plot
    subplot(1,3,3);
    t_jerk = times(1:end-3);
    plot(t_jerk, original_jerk_norm, 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t_jerk, smoothed_jerk_norm, 'b-', 'LineWidth', 1.5);
    title('Joint Jerk Magnitude');
    xlabel('Time (s)');
    ylabel('Jerk (rad/s³)');
    legend('Original', 'Smoothed');
    grid on;
    
    % Add title with metrics
    sgtitle(sprintf('Derivative Analysis: Accel. Reduction: %.1f%%, Jerk Reduction: %.1f%%', ...
            accel_reduction, jerk_reduction), 'FontWeight', 'bold');
end