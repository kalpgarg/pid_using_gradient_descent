% Define plant with Pade approximation for ZOH
sample_time_normal = 0.6;
zoh_pade_normal = tf([1], [sample_time_normal/2 1]);

plant_K = 0.99;
plant_zeta = 0.62;
plant_ts = 0.05;
plant_deadTime_normal = 0.45;

plant_num = [plant_K];
plant_deno = [plant_ts^2, 2*plant_zeta*plant_ts, 1];
G1_orig = tf(plant_num, plant_deno, 'IODelay', plant_deadTime_normal);
G = G1_orig * zoh_pade_normal;

% Step 1: Find Ku and Tu (using proportional-only controller)
fprintf('Searching for ultimate gain (Ku) and period (Tu)...\n');
Kp = 0.1;
Ku = NaN;
Tu = NaN;

while Kp < 100
    C = pid(Kp, 0, 0);           % Proportional-only controller
    T = feedback(C * G, 1);      % Closed-loop transfer function
    [y, t] = step(T, 30);        % Step response over 30s
    y = y - mean(y(end-10:end)); % Remove steady state to detect oscillation

    % Detect peaks
    [pks, locs] = findpeaks(y, t, 'MinPeakProminence', 0.01);
    
    % Check for sustained oscillations
    if length(pks) >= 6
        % Check if peak heights are roughly constant (±10%)
        pk_ratio = max(pks) / min(pks);
        if pk_ratio < 1.1
            Ku = Kp;
            Tu = mean(diff(locs(1:6))); % Use first 6 peaks for Tu
            fprintf('Sustained oscillations detected.\nKu = %.4f\nTu = %.4f s\n', Ku, Tu);
            break;
        end
    end
    Kp = Kp + 0.5;
end

if isnan(Ku)
    error('Failed to find Ku and Tu. Try adjusting gain range or time.');
end

% Step 2: Apply Ziegler-Nichols PID tuning formulas
Kp_pid = 0.6 * Ku;
Ki_pid = 2 * Kp_pid / Tu;
Kd_pid = Kp_pid * Tu / 8;

fprintf('\nZiegler-Nichols PID gains:\n');
fprintf('Kp = %.4f\n', Kp_pid);
fprintf('Ki = %.4f\n', Ki_pid);
fprintf('Kd = %.4f\n', Kd_pid);

% Optional: Plot step response with PID controller
C_pid = pid(Kp_pid, Ki_pid, Kd_pid);
T_pid = feedback(C_pid * G, 1);
figure;
step(T_pid);
title('Closed-loop Step Response with Z-N method');
grid on;