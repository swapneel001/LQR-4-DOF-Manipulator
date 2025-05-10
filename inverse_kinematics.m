function theta = inverse_kinematics(target_pos, current_theta, options)
    % Import kinematics module
    [fwd_kin, jac_fn] = kinematics();
    
    % Default options
    if nargin < 3
        options.tolerance = 1e-4;
        options.max_iterations = 5;  % Number of linearization steps
    end
    
    % Current configuration
    theta = current_theta;
    
    % Ensure theta is a column vector
    if size(theta, 2) > size(theta, 1)
        theta = theta';
    end
    
    % Check if target is within maximum reach
    max_reach = 10;  % Sum of all link lengths
    target_dist = norm(target_pos);
    
    if target_dist > max_reach * 0.95
        fprintf('Warning: Target (%.2f, %.2f) close to or beyond max reach (%.2f)\n', ...
                target_pos(1), target_pos(2), max_reach);
        % Scale back to 95% of max reach in same direction
        if target_dist > max_reach
            target_pos = target_pos * (max_reach * 0.95) / target_dist;
            fprintf('Adjusted target to (%.2f, %.2f)\n', target_pos(1), target_pos(2));
        end
    end
    
    % Iterative linearization and optimization
    for iter = 1:options.max_iterations
        % Current position and error
        [x, y] = fwd_kin(theta);
        current_pos = [x; y];
        error_vec = target_pos - current_pos;
        error_norm = norm(error_vec);
        
        % Check if we've converged
        if error_norm < options.tolerance
            fprintf('Converged after %d iterations with error %.6f\n', iter, error_norm);
            break;
        end
        
        % Get Jacobian at current configuration
        J = jac_fn(theta);
        
        % Check for NaN in Jacobian (indicating a singular configuration)
        if any(isnan(J(:)))
            fprintf('Warning: NaN detected in Jacobian, using fallback method\n');
            % Use a simple Jacobian transpose method as fallback
            delta_theta = 0.1 * J' * error_vec;
            if norm(delta_theta) > 0.5
                delta_theta = 0.5 * delta_theta / norm(delta_theta);
            end
            theta = theta + delta_theta;
            continue;
        end
        
        % Attempt direct computation (pseudoinverse) first
        % This is simpler and more stable than optimization
        if rank(J) == size(J, 1)  % Full row rank
            % Use pseudoinverse for better numerical stability
            delta_theta = J' * ((J * J' + 1e-6 * eye(size(J, 1))) \ error_vec);
            
            % Limit step size
            if norm(delta_theta) > 0.5
                delta_theta = 0.5 * delta_theta / norm(delta_theta);
            end
            
            theta = theta + delta_theta;
            fprintf('Used pseudoinverse for iteration %d\n', iter);
            continue;
        end
        
        % Fallback to Jacobian transpose method if pseudoinverse fails
        % This should always work but may be slower to converge
        delta_theta = 0.1 * J' * error_vec;
        if norm(delta_theta) > 0.5
            delta_theta = 0.5 * delta_theta / norm(delta_theta);
        end
        theta = theta + delta_theta;
        fprintf('Used Jacobian transpose for iteration %d\n', iter);
    end
    
    % Final verification
    [x_final, y_final] = fwd_kin(theta);
    final_pos = [x_final; y_final];
    final_error = norm(final_pos - target_pos);
    
    fprintf('Final position: (%.4f, %.4f), Error: %.6f\n', ...
            final_pos(1), final_pos(2), final_error);
            
    % Check for NaN in result
    if any(isnan(theta))
        fprintf('ERROR: NaN in solution, using simple initial guess\n');
        % Try a simple guess based on target direction
        theta = [atan2(target_pos(2), target_pos(1)); 0; 0; 0];
        
        % Try to improve this guess with one more iteration
        J = jac_fn(theta);
        [x, y] = fwd_kin(theta);
        error_vec = target_pos - [x; y];
        delta_theta = 0.2 * J' * error_vec;
        theta = theta + delta_theta;
    end
end