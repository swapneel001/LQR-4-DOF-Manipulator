function metrics = analysis(t, x, y, u, reference, original_reference)
    % Analyze controller performance with enhanced smoothing analysis
    % Additional optional input: original_reference for comparison
    
    metrics = struct();
    
    % Calculate tracking error if reference is provided
    if nargin > 4
        % Interpolate reference to match simulation time points
        t_ref = reference.t;
        y_ref = reference.y;
        
        % Interpolate reference trajectory to match simulation time points
        y_ref_interp = zeros(size(y));
        for i = 1:size(y,1)
            y_ref_interp(i,:) = interp1(t_ref, y_ref(i,:), t, 'linear', 'extrap');
        end
        
        % Tracking error
        error = y - y_ref_interp;
        
        % Position tracking error (Euclidean distance)
        pos_error = sqrt(error(1,:).^2 + error(2,:).^2);
        
        % RMS error
        metrics.rms_error_x = rms(error(1,:));
        metrics.rms_error_y = rms(error(2,:));
        metrics.rms_error_pos = rms(pos_error);
        
        % Maximum error
        metrics.max_error_x = max(abs(error(1,:)));
        metrics.max_error_y = max(abs(error(2,:)));
        metrics.max_error_pos = max(pos_error);
        
        % Average error
        metrics.avg_error_x = mean(abs(error(1,:)));
        metrics.avg_error_y = mean(abs(error(2,:)));
        metrics.avg_error_pos = mean(pos_error);
        
        % Plot tracking error
        figure('Name', 'Tracking Error Analysis', 'Position', [100, 100, 1000, 700]);
        
        subplot(3,1,1);
        plot(t, error(1:2,:)', 'LineWidth', 1.5);
        title('Position Tracking Error');
        xlabel('Time (s)');
        ylabel('Error (m)');
        legend('X Error', 'Y Error');
        grid on;
        
        subplot(3,1,2);
        plot(t, pos_error, 'LineWidth', 1.5);
        title('Euclidean Position Error');
        xlabel('Time (s)');
        ylabel('Error (m)');
        grid on;
        
        % Add original reference comparison if provided
        if nargin > 5 && ~isempty(original_reference)
            % Interpolate original reference
            t_orig_ref = original_reference.t;
            y_orig_ref = original_reference.y;
            
            y_orig_ref_interp = zeros(size(y));
            for i = 1:size(y,1)
                y_orig_ref_interp(i,:) = interp1(t_orig_ref, y_orig_ref(i,:), t, 'linear', 'extrap');
            end
            
            % Original vs Smoothed reference comparison
            subplot(3,1,3);
            ref_diff = y_ref_interp(1:2,:) - y_orig_ref_interp(1:2,:);
            plot(t, vecnorm(ref_diff), 'g-', 'LineWidth', 1.5);
            hold on;
            
            % Highlight regions where smoothing might affect tracking
            threshold = mean(vecnorm(ref_diff)) * 2;
            high_diff_indices = vecnorm(ref_diff) > threshold;
            if any(high_diff_indices)
                plot(t(high_diff_indices), vecnorm(ref_diff(:,high_diff_indices)), 'r.', 'MarkerSize', 10);
            end
            
            title('Original vs. Smoothed Trajectory Deviation');
            xlabel('Time (s)');
            ylabel('Deviation (m)');
            grid on;
            
            % Calculate smoothing metrics
            metrics.original_vs_smoothed_max_dev = max(vecnorm(ref_diff));
            metrics.original_vs_smoothed_avg_dev = mean(vecnorm(ref_diff));
            metrics.original_vs_smoothed_rms_dev = rms(vecnorm(ref_diff));
        end
    end
    
    % Control effort analysis
    dt = t(2) - t(1);
    
    % Total control effort (sum of squared inputs)
    metrics.total_effort = sum(sum(u.^2))*dt;
    
    % Maximum control input
    metrics.max_control = max(max(abs(u)));
    
    % Average control magnitude
    metrics.avg_control = mean(mean(abs(u)));
    
    % Plot control effort
    figure('Name', 'Control Effort Analysis', 'Position', [150, 150, 900, 600]);
    
    subplot(2,1,1);
    stairs(t(1:end-1), vecnorm(u), 'LineWidth', 1.5);
    title('Control Effort Magnitude');
    xlabel('Time (s)');
    ylabel('Control Magnitude (Nm)');
    grid on;
    
    % Calculate control effort spectrum
    subplot(2,1,2);
    Fs = 1/dt;
    L = size(u,2);
    f = Fs*(0:(L/2))/L;
    
    % FFT of control signals
    control_fft = zeros(4, floor(L/2)+1);
    for i = 1:4
        Y = fft(u(i,:));
        P2 = abs(Y/L);
        control_fft(i,:) = P2(1:floor(L/2)+1);
        control_fft(i,2:end-1) = 2*control_fft(i,2:end-1);
    end
    
    semilogy(f, control_fft', 'LineWidth', 1.5);
    title('Control Effort Frequency Spectrum');
    xlabel('Frequency (Hz)');
    ylabel('Magnitude');
    legend('\tau_1', '\tau_2', '\tau_3', '\tau_4');
    grid on;
    xlim([0 Fs/10]); % Show only up to 1/10 of Nyquist frequency for clarity
    
    % Calculate additional spectral metrics
    metrics.control_high_freq_content = sum(control_fft(:,floor(L/20):end), 'all') / sum(control_fft, 'all');
    
    % Print summary
    fprintf('\n===== Controller Performance Metrics =====\n');
    if nargin > 4
        fprintf('Position Tracking:\n');
        fprintf('  RMS Error: %.4f m\n', metrics.rms_error_pos);
        fprintf('  Max Error: %.4f m\n', metrics.max_error_pos);
        fprintf('  Avg Error: %.4f m\n', metrics.avg_error_pos);
        
        if nargin > 5 && ~isempty(original_reference)
            fprintf('\nTrajectory Comparison (Original vs. Smoothed):\n');
            fprintf('  Max Deviation: %.4f m\n', metrics.original_vs_smoothed_max_dev);
            fprintf('  Avg Deviation: %.4f m\n', metrics.original_vs_smoothed_avg_dev);
            fprintf('  RMS Deviation: %.4f m\n', metrics.original_vs_smoothed_rms_dev);
        end
    end
    
    fprintf('\nControl Effort:\n');
    fprintf('  Total Effort: %.4f\n', metrics.total_effort);
    fprintf('  Max Control: %.4f Nm\n', metrics.max_control);
    fprintf('  Avg Control: %.4f Nm\n', metrics.avg_control);
    fprintf('  High Frequency Content: %.2f%%\n', metrics.control_high_freq_content * 100);
    fprintf('========================================\n\n');
    
    % Visualize joint trajectories if available
    if nargin > 1
        figure('Name', 'Joint Trajectories', 'Position', [200, 200, 900, 600]);
        
        % Joint positions
        subplot(2,1,1);
        plot(t, x(1:4,:)', 'LineWidth', 1.5);
        title('Joint Angles');
        xlabel('Time (s)');
        ylabel('Angle (rad)');
        legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
        grid on;
        
        % Joint velocities
        subplot(2,1,2);
        plot(t, x(5:8,:)', 'LineWidth', 1.5);
        title('Joint Velocities');
        xlabel('Time (s)');
        ylabel('Angular Velocity (rad/s)');
        legend('d\theta_1/dt', 'd\theta_2/dt', 'd\theta_3/dt', 'd\theta_4/dt');
        grid on;
    end
    
    % Enhanced smoothing analysis
    % If both references are available, create trajectory comparison plot
    if nargin > 5 && ~isempty(original_reference)
        % Create acceleration comparison plot
        figure('Name', 'Smoothing Analysis', 'Position', [250, 250, 1200, 700]);
        
        % Calculate accelerations from both trajectories
        % First, resample to same time vector if needed
        if length(reference.t) ~= length(original_reference.t) || any(reference.t ~= original_reference.t)
            t_uniform = linspace(min(reference.t(1), original_reference.t(1)), ...
                                max(reference.t(end), original_reference.t(end)), ...
                                length(reference.t));
                            
            % Resample joint angles
            smoothed_q = zeros(4, length(t_uniform));
            original_q = zeros(4, length(t_uniform));
            
            for i = 1:4
                smoothed_q(i,:) = interp1(reference.t, reference.q(i,:), t_uniform, 'spline');
                original_q(i,:) = interp1(original_reference.t, original_reference.q(i,:), t_uniform, 'spline');
            end
        else
            t_uniform = reference.t;
            smoothed_q = reference.q;
            original_q = original_reference.q;
        end
        
        % Calculate accelerations
        dt_uniform = t_uniform(2) - t_uniform(1);
        
        smoothed_vel = zeros(4, length(t_uniform)-1);
        original_vel = zeros(4, length(t_uniform)-1);
        
        for i = 1:length(t_uniform)-1
            smoothed_vel(:,i) = (smoothed_q(:,i+1) - smoothed_q(:,i))/dt_uniform;
            original_vel(:,i) = (original_q(:,i+1) - original_q(:,i))/dt_uniform;
        end
        
        smoothed_accel = zeros(4, length(t_uniform)-2);
        original_accel = zeros(4, length(t_uniform)-2);
        
        for i = 1:length(t_uniform)-2
            smoothed_accel(:,i) = (smoothed_vel(:,i+1) - smoothed_vel(:,i))/dt_uniform;
            original_accel(:,i) = (original_vel(:,i+1) - original_vel(:,i))/dt_uniform;
        end
        
        % Calculate jerk
        smoothed_jerk = zeros(4, length(t_uniform)-3);
        original_jerk = zeros(4, length(t_uniform)-3);
        
        for i = 1:length(t_uniform)-3
            smoothed_jerk(:,i) = (smoothed_accel(:,i+1) - smoothed_accel(:,i))/dt_uniform;
            original_jerk(:,i) = (original_accel(:,i+1) - original_accel(:,i))/dt_uniform;
        end
        
        % Plot accelerations
        t_accel = t_uniform(1:end-2);
        t_jerk = t_uniform(1:end-3);
        
        % Create 2x2 subplot layout for comprehensive smoothing analysis
        subplot(2,2,1);
        smoothed_accel_norm = vecnorm(smoothed_accel);
        original_accel_norm = vecnorm(original_accel);
        
        plot(t_accel, original_accel_norm, 'r-', 'LineWidth', 1.5);
        hold on;
        plot(t_accel, smoothed_accel_norm, 'b-', 'LineWidth', 1.5);
        
        title('Joint Acceleration Magnitude');
        xlabel('Time (s)');
        ylabel('Acceleration (rad/s²)');
        legend('Original', 'Smoothed');
        grid on;
        
        % Plot jerk
        subplot(2,2,2);
        smoothed_jerk_norm = vecnorm(smoothed_jerk);
        original_jerk_norm = vecnorm(original_jerk);
        
        plot(t_jerk, original_jerk_norm, 'r-', 'LineWidth', 1.5);
        hold on;
        plot(t_jerk, smoothed_jerk_norm, 'b-', 'LineWidth', 1.5);
        
        title('Joint Jerk Magnitude');
        xlabel('Time (s)');
        ylabel('Jerk (rad/s³)');
        legend('Original', 'Smoothed');
        grid on;
        
        % Calculate acceleration and jerk reduction
        accel_reduction = (mean(original_accel_norm) - mean(smoothed_accel_norm)) / mean(original_accel_norm) * 100;
        jerk_reduction = (mean(original_jerk_norm) - mean(smoothed_jerk_norm)) / mean(original_jerk_norm) * 100;
        
        % Plot velocity comparison
        subplot(2,2,3);
        t_vel = t_uniform(1:end-1);
        smoothed_vel_norm = vecnorm(smoothed_vel);
        original_vel_norm = vecnorm(original_vel);
        
        plot(t_vel, original_vel_norm, 'r-', 'LineWidth', 1.5);
        hold on;
        plot(t_vel, smoothed_vel_norm, 'b-', 'LineWidth', 1.5);
        
        title('Joint Velocity Magnitude');
        xlabel('Time (s)');
        ylabel('Velocity (rad/s)');
        legend('Original', 'Smoothed');
        grid on;
        
        % Velocity reduction
        vel_reduction = (mean(original_vel_norm) - mean(smoothed_vel_norm)) / mean(original_vel_norm) * 100;
        
        % Plot deviation between trajectories
        subplot(2,2,4);
        t_pos = t_uniform;
        position_deviation = zeros(1, length(t_pos));
        
        % Get original and smoothed end-effector positions
        [fwd_kin, ~] = kinematics();
        smoothed_pos = zeros(2, length(t_pos));
        original_pos = zeros(2, length(t_pos));
        
        for i = 1:length(t_pos)
            [x_smooth, y_smooth] = fwd_kin(smoothed_q(:,i));
            [x_orig, y_orig] = fwd_kin(original_q(:,i));
            smoothed_pos(:,i) = [x_smooth; y_smooth];
            original_pos(:,i) = [x_orig; y_orig];
            position_deviation(i) = norm(smoothed_pos(:,i) - original_pos(:,i));
        end
        
        plot(t_pos, position_deviation, 'g-', 'LineWidth', 1.5);
        title('Position Deviation Due to Smoothing');
        xlabel('Time (s)');
        ylabel('Deviation (m)');
        grid on;
        
        % Calculate smoothing metrics
        position_deviation_max = max(position_deviation);
        position_deviation_avg = mean(position_deviation);
        position_deviation_rms = rms(position_deviation);
        
        % Add title with metrics
        sgtitle(sprintf('Smoothing Analysis: Accel. Reduction: %.1f%%, Jerk Reduction: %.1f%%', ...
                accel_reduction, jerk_reduction), 'FontWeight', 'bold');
        
        % Store metrics
        metrics.accel_reduction_pct = accel_reduction;
        metrics.jerk_reduction_pct = jerk_reduction;
        metrics.vel_reduction_pct = vel_reduction;
        metrics.original_accel_rms = rms(original_accel_norm);
        metrics.smoothed_accel_rms = rms(smoothed_accel_norm);
        metrics.original_jerk_rms = rms(original_jerk_norm);
        metrics.smoothed_jerk_rms = rms(smoothed_jerk_norm);
        metrics.position_deviation_max = position_deviation_max;
        metrics.position_deviation_avg = position_deviation_avg;
        metrics.position_deviation_rms = position_deviation_rms;
        
        % Create frequency domain analysis
        figure('Name', 'Smoothing Frequency Analysis', 'Position', [300, 300, 1000, 800]);
        
        % Calculate FFT for both signals
        Fs = 1/dt_uniform;
        for joint = 1:4
            subplot(2,2,joint);
            
            % Compute FFT
            L = length(t_uniform);
            f = Fs*(0:(L/2))/L;
            
            % Original trajectory FFT
            Y_orig = fft(original_q(joint,:));
            P2_orig = abs(Y_orig/L);
            P1_orig = P2_orig(1:floor(L/2)+1);
            P1_orig(2:end-1) = 2*P1_orig(2:end-1);
            
            % Smoothed trajectory FFT
            Y_smooth = fft(smoothed_q(joint,:));
            P2_smooth = abs(Y_smooth/L);
            P1_smooth = P2_smooth(1:floor(L/2)+1);
            P1_smooth(2:end-1) = 2*P1_smooth(2:end-1);
            
            % Plot frequency spectrum
            semilogy(f, P1_orig, 'r-', 'LineWidth', 1.5);
            hold on;
            semilogy(f, P1_smooth, 'b-', 'LineWidth', 1.5);
            
            title(sprintf('Joint %d Frequency Spectrum', joint));
            xlabel('Frequency (Hz)');
            ylabel('Magnitude');
            legend('Original', 'Smoothed');
            grid on;
            xlim([0 Fs/10]); % Show only up to 1/10 of Nyquist frequency for clarity
            
            % Calculate high frequency energy reduction
            high_freq_idx = f > 1.0; % Frequencies above 1 Hz
            if any(high_freq_idx)
                high_freq_energy_orig = sum(P1_orig(high_freq_idx));
                high_freq_energy_smooth = sum(P1_smooth(high_freq_idx));
                high_freq_reduction = (high_freq_energy_orig - high_freq_energy_smooth) / high_freq_energy_orig * 100;
                
                % Store high frequency metrics
                metrics.(['joint', num2str(joint), '_high_freq_reduction_pct']) = high_freq_reduction;
            end
        end
        
        % Print smoothing performance metrics
        fprintf('\n===== Smoothing Performance Metrics =====\n');
        fprintf('Trajectory Deviation:\n');
        fprintf('  Maximum Deviation: %.4f m\n', position_deviation_max);
        fprintf('  Average Deviation: %.4f m\n', position_deviation_avg);
        fprintf('  RMS Deviation: %.4f m\n', position_deviation_rms);
        
        fprintf('\nDerivative Reductions:\n');
        fprintf('  Velocity Reduction: %.2f%%\n', vel_reduction);
        fprintf('  Acceleration Reduction: %.2f%%\n', accel_reduction);
        fprintf('  Jerk Reduction: %.2f%%\n', jerk_reduction);
        
        fprintf('\nFrequency Domain Analysis:\n');
        for joint = 1:4
            if isfield(metrics, ['joint', num2str(joint), '_high_freq_reduction_pct'])
                fprintf('  Joint %d High Frequency Reduction: %.2f%%\n', joint, metrics.(['joint', num2str(joint), '_high_freq_reduction_pct']));
            end
        end
        fprintf('=======================================\n\n');
    end
end