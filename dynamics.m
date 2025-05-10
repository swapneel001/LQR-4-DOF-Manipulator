function [A, B, C, D] = dynamics(theta_operating)
    % Create state-space model for the 4-DOF manipulator
    
    % Get Jacobian at operating point
    [~, jac_fn] = kinematics();
    J = jac_fn(theta_operating);
    
    % System dimensions
    n = 4;  % Number of joints
    
    % A matrix (double integrator model)
    A = [zeros(n), eye(n);
         zeros(n), zeros(n)];
    
    % B matrix (control inputs to joint accelerations)
    B = [zeros(n);
         eye(n)];
    
    % C matrix (output is end-effector position and velocity)
    C = zeros(4, 2*n);
    C(1:2, 1:n) = J;      % Position from joint angles
    C(3:4, n+1:2*n) = J;  % Velocity from joint velocities
    
    % D matrix (no direct feedthrough)
    D = zeros(4, n);
end