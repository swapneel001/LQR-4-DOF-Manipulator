function visualization(t, x, y, u, trajectory)
    % Visualize simulation results
    
    % Create figure for state trajectories
    figure('Name', 'Simulation Results', 'Position', [100, 100, 1000, 800]);
    
    % Plot joint angles
    subplot(3,1,1);
    plot(t, x(1:4,:), 'LineWidth', 1.5);
    hold on;
    if nargin > 4
        % Plot reference joint angles if available
        t_ref = trajectory.t;
        q_ref = trajectory.q;
        colors = get(gca, 'ColorOrder');
        for i = 1:4
            plot(t_ref, q_ref(i,:), '--', 'Color', colors(i,:));
        end
    end
    title('Joint Angles');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', ...
           '\theta_1 ref', '\theta_2 ref', '\theta_3 ref', '\theta_4 ref');
    grid on;
    
    % Plot end-effector position
    subplot(3,1,2);
    plot(t, y(1:2,:), 'LineWidth', 1.5);
    hold on;
    if nargin > 4
        % Plot reference end-effector position
        plot(t_ref, trajectory.y(1,:), '--', 'LineWidth', 1.5);
        plot(t_ref, trajectory.y(2,:), '--', 'LineWidth', 1.5);
    end
    title('End-Effector Position');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('x_{ee}', 'y_{ee}', 'x_{ee} ref', 'y_{ee} ref');
    grid on;
    
    % Plot control inputs
    subplot(3,1,3);
    plot(t(1:end-1), u, 'LineWidth', 1.5);
    title('Control Inputs (Joint Torques)');
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    legend('\tau_1', '\tau_2', '\tau_3', '\tau_4');
    grid on;
    
    % Plot task-space trajectory
    figure('Name', 'Task-Space Trajectory', 'Position', [200, 200, 800, 600]);
    
    % Get kinematics functions for workspace plotting
    [fwd_kin, ~] = kinematics();
    
    % Extract actual end-effector trajectory
    ee_x = y(1,:);
    ee_y = y(2,:);
    
    % Extract reference trajectory
    ref_x = trajectory.y(1,:);
    ref_y = trajectory.y(2,:);
    
    % Plot trajectories
    plot(ee_x, ee_y, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    hold on;
    plot(ref_x, ref_y, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
    
    % Extract and plot waypoints
    % Try to identify waypoints from the trajectory
    [~, idx] = findpeaks(abs(diff(ref_x)), 'MinPeakHeight', 0.1);
    if length(idx) < 4 % If can't find peaks, try from velocity
        vel_x = diff(ref_x);
        vel_y = diff(ref_y);
        speed = sqrt(vel_x.^2 + vel_y.^2);
        [~, idx] = findpeaks(-speed, 'MinPeakHeight', -0.1);
    end
    
    % If still can't find peaks, just use evenly spaced points
    if length(idx) < 4
        idx = round(linspace(1, length(ref_x), 5));
    end
    
    % Plot waypoints
    plot(ref_x(idx), ref_y(idx), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Waypoints');
    
    % Check for original trajectory file
    if exist('original_trajectory.mat', 'file')
        try
            % Load the original trajectory if available
            orig_data = load('original_trajectory.mat');
            if isfield(orig_data, 'original_traj')
                original_traj = orig_data.original_traj;
                plot(original_traj.y(1,:), original_traj.y(2,:), 'm-.', 'LineWidth', 1.5, 'DisplayName', 'Original Unsmoothed');
                
                % Calculate path deviation
                min_len = min(length(ref_x), length(original_traj.y(1,:)));
                path_deviation = zeros(1, min_len);
                for i = 1:min_len
                    path_deviation(i) = norm([ref_x(i); ref_y(i)] - original_traj.y(1:2,i));
                end
                
                max_deviation = max(path_deviation);
                avg_deviation = mean(path_deviation);
                
                fprintf('\nTrajectory Smoothing Metrics:\n');
                fprintf('  Maximum path deviation: %.4f m\n', max_deviation);
                fprintf('  Average path deviation: %.4f m\n', avg_deviation);
            end
        catch
            % Ignore errors if file can't be loaded
        end
    end
    
    % Plot workspace boundary
    theta_circle = linspace(0, 2*pi, 100);
    max_reach = 10;  % Sum of link lengths
    plot(max_reach*cos(theta_circle), max_reach*sin(theta_circle), 'k--', 'DisplayName', 'Workspace Limit');
    
    % Calculate actual trajectory error
    error_x = ee_x - ref_x;
    error_y = ee_y - ref_y;
    error_norm = sqrt(error_x.^2 + error_y.^2);
    max_error = max(error_norm);
    avg_error = mean(error_norm);
    
    % Add title with error metrics
    title(sprintf('Task-Space Trajectory (Max Error: %.4f m, Avg Error: %.4f m)', max_error, avg_error));
    
    % Add labels and legend
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend('Location', 'best');
    grid on;
    axis equal;
    
    % Create animation
    animate_robot(t, x, 5);  % Speed up animation 5x
end

function animate_robot(t, x, speed_factor)
    % Create animation of the 4-DOF planar robot
    
    % Extract joint angles
    theta = x(1:4,:);
    
    % Link lengths
    L = [4, 3, 2, 1];
    
    % Calculate frame interval based on speed factor
    interval = max(1, round(length(t)/(100*speed_factor)));
    
    % Setup figure
    figure('Name', 'Robot Animation', 'Position', [300, 300, 800, 600]);
    hold on;
    grid on;
    axis equal;
    
    % Set axis limits based on workspace
    axis([-11 11 -11 11]);
    
    % Draw workspace boundary
    theta_circle = linspace(0, 2*pi, 100);
    max_reach = sum(L);
    plot(max_reach*cos(theta_circle), max_reach*sin(theta_circle), 'r--');
    
    % Initialize robot links
    h_links = gobjects(4,1);
    for i = 1:4
        h_links(i) = plot([0,0], [0,0], 'LineWidth', 2);
    end
    
    % Initialize end-effector marker
    h_ee = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Initialize path trace
    h_path = plot(0, 0, 'b-', 'LineWidth', 1);
    path_x = [];
    path_y = [];
    
    % Animation loop
    for i = 1:interval:length(t)
        % Current joint angles
        q = theta(:,i);
        
        % Forward kinematics
        p0 = [0; 0];  % Base at origin
        p1 = p0 + L(1)*[cos(q(1)); sin(q(1))];
        p2 = p1 + L(2)*[cos(q(1)+q(2)); sin(q(1)+q(2))];
        p3 = p2 + L(3)*[cos(q(1)+q(2)+q(3)); sin(q(1)+q(2)+q(3))];
        p4 = p3 + L(4)*[cos(q(1)+q(2)+q(3)+q(4)); sin(q(1)+q(2)+q(3)+q(4))];
        
        % Update robot link positions
        set(h_links(1), 'XData', [p0(1), p1(1)], 'YData', [p0(2), p1(2)]);
        set(h_links(2), 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)]);
        set(h_links(3), 'XData', [p2(1), p3(1)], 'YData', [p2(2), p3(2)]);
        set(h_links(4), 'XData', [p3(1), p4(1)], 'YData', [p3(2), p4(2)]);
        
        % Update end-effector
        set(h_ee, 'XData', p4(1), 'YData', p4(2));
        
        % Update path trace
        path_x = [path_x, p4(1)];
        path_y = [path_y, p4(2)];
        set(h_path, 'XData', path_x, 'YData', path_y);
        
        % Title with time
        title(sprintf('Robot Animation - Time: %.2f s', t(i)));
        
        % Draw and pause
        drawnow;
        pause(0.01);
    end
end