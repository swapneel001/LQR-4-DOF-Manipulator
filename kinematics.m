function [fwd, jac] = kinematics()
    % Create function handles for forward kinematics and Jacobian
    
    % Link lengths
    L = [4, 3, 2, 1];
    
    % Forward kinematics function
    function [x, y] = forward_kin(theta)
        x = L(1)*cos(theta(1)) + L(2)*cos(theta(1)+theta(2)) + ...
            L(3)*cos(theta(1)+theta(2)+theta(3)) + L(4)*cos(sum(theta));
        y = L(1)*sin(theta(1)) + L(2)*sin(theta(1)+theta(2)) + ...
            L(3)*sin(theta(1)+theta(2)+theta(3)) + L(4)*sin(sum(theta));
    end

    % Jacobian function
    function J = jacobian(theta)
        J = zeros(2, 4);
        
        % Precompute sines and cosines
        c1 = cos(theta(1));
        s1 = sin(theta(1));
        c12 = cos(theta(1)+theta(2));
        s12 = sin(theta(1)+theta(2));
        c123 = cos(theta(1)+theta(2)+theta(3));
        s123 = sin(theta(1)+theta(2)+theta(3));
        c1234 = cos(sum(theta));
        s1234 = sin(sum(theta));
        
        % x-coordinate Jacobian elements (dx/dθᵢ)
        J(1,1) = -L(1)*s1 - L(2)*s12 - L(3)*s123 - L(4)*s1234;
        J(1,2) = -L(2)*s12 - L(3)*s123 - L(4)*s1234;
        J(1,3) = -L(3)*s123 - L(4)*s1234;
        J(1,4) = -L(4)*s1234;
        
        % y-coordinate Jacobian elements (dy/dθᵢ)
        J(2,1) = L(1)*c1 + L(2)*c12 + L(3)*c123 + L(4)*c1234;
        J(2,2) = L(2)*c12 + L(3)*c123 + L(4)*c1234;
        J(2,3) = L(3)*c123 + L(4)*c1234;
        J(2,4) = L(4)*c1234;
    end

    % Return function handles
    fwd = @forward_kin;
    jac = @jacobian;
end