function [t, x, y, u] = robot_sim(trajectory, controller_K, options)
    % Simulate the 4-DOF manipulator with LQR control
    
    % Default options
    if nargin < 3
        options.t_final = 10;
        options.dt = 0.01;
        options.disturbances = false;
        options.dist_magnitude = 0.1;
    end
    
    % Create time vector
    t = 0:options.dt:options.t_final;
    n_steps = length(t);
    
    % State dimension
    n_joints = 4;
    n_states = 2*n_joints;  % angles and velocities
    n_inputs = n_joints;    % torques
    
    % Initialize arrays
    x = zeros(n_states, n_steps);
    y = zeros(4, n_steps);  % Position (2) and velocity (2)
    u = zeros(n_inputs, n_steps-1);
    
    % Set initial state (from trajectory)
    t_traj = trajectory.t;
    q_traj = trajectory.q;
    dq_traj = trajectory.dq;
    
    % Find initial state from trajectory
    x(:,1) = [q_traj(:,1); dq_traj(:,1)];
    
    % Import kinematics functions
    [fwd_kin, jac_fn] = kinematics();
    
    % Main simulation loop
    for k = 1:(n_steps-1)
        % Current time and state
        t_current = t(k);
        x_current = x(:,k);
        
        % Find reference state from trajectory (closest time point)
        [~, idx] = min(abs(t_traj - t_current));
        x_ref = [q_traj(:,idx); dq_traj(:,idx)];
        
        % Linearize system around current operating point
        [A, B, C, D] = dynamics(x_current(1:n_joints));
        
        % Compute control input (LQR state feedback)
        u(:,k) = -controller_K * (x_current - x_ref);
        
        % Add optional disturbances
        if isfield(options, 'disturbances') && options.disturbances
            u(:,k) = u(:,k) + options.dist_magnitude * randn(n_inputs, 1);
        end
        
        % Apply input constraints if needed
        u(:,k) = min(max(u(:,k), -10), 10);  % Limit torques to ±10 N·m
        
        % Update state using linearized dynamics
        x_dot = A*x_current + B*u(:,k);
        x(:,k+1) = x_current + options.dt * x_dot;
        
        % Compute end-effector position and velocity for output
        [pos_x, pos_y] = fwd_kin(x(1:4,k));
        y(1:2,k) = [pos_x; pos_y];
        
        % Compute velocity using Jacobian from dynamics
        J = jac_fn(x(1:4,k));
        y(3:4,k) = J * x(5:8,k);  % Velocity
    end
    
    % Compute final output
    [pos_x, pos_y] = fwd_kin(x(1:4,end));
    y(1:2,end) = [pos_x; pos_y];
    J = jac_fn(x(1:4,end));
    y(3:4,end) = J * x(5:8,end);
    
    % Print simulation summary
    fprintf('Simulation completed for %.2f seconds with %d steps\n', options.t_final, n_steps);
end