function [K, info] = lqr_controller(A, B, Q, R)
    % Design LQR controller for the 4-DOF manipulator
    
    % Default weighting matrices if not provided
    if nargin < 3
        % Higher weights for position states than velocity states
        Q = diag([10, 10, 10, 10, 1, 1, 1, 1]);
    end
    
    if nargin < 4
        % Penalize large control efforts
        R = eye(4);
    end
    
    % Compute LQR gain matrix
    [K, S, e] = lqr(A, B, Q, R);
    disp('K is ')
    K
    % Return additional information
    info.S = S;  % Solution to Riccati equation
    info.e = e;  % Closed-loop eigenvalues
    info.closed_loop_A = A - B*K;
    
    % Analyze controller performance
    fprintf('LQR Controller designed successfully\n');
    fprintf('Closed-loop eigenvalues:\n');
    disp(e);
    
    % Estimate settling time
    settling_time = -4/max(real(e));
    fprintf('Estimated settling time: %.2f seconds\n', settling_time);
    info.settling_time = settling_time;
    
    % Check stability
    if all(real(e) < 0)
        fprintf('Closed-loop system is stable\n');
    else
        warning('Closed-loop system may be unstable!');
    end
end