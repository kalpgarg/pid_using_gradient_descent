
% Initial parameters
Kp = 0.4; Ki = 1.2; Kd = 0.1; alpha = 0.0005;
num_iterations = 1000; tolerance = 1e-4;
divg_limit = 50;
% Initial PID gains
% Learning rate for gradient descent % Number of gradient descent iterations % Convergence tolerance
% Divergence limit
% Weights for cost function J
w1 = 1;
w2 = 1;
% Weight for settling time
% Weight for maximum overshoot
%% Define plant
sample_time_normal = 0.6;
zoh_pade_normal = tf([1], [sample_time_normal/2 1]);
plant_K = 0.99; plant_zeta = 0.62; plant_ts = 0.05; plant_deadTime_normal = 0.45;
% plant_K = 1.2; plant_zeta = 0.38; plant_ts = 0.025; plant_deadTime_normal = 0.305;
plant_num = [plant_K];
plant_deno = [plant_ts*plant_ts 2*plant_zeta*plant_ts 1];
G1_orig = tf(plant_num, plant_deno, 'IODelay', plant_deadTime_normal);
G= G1_orig*zoh_pade_normal;
% Initialize cost function
J_history = zeros(num_iterations, 1);

% Gradient Descent Loop
for i = 1:num_iterations
    % Construct PID controller with current gains
    K = pid (Kp, Ki, Kd);
    %% Closed-loop system
    ClosedLoop = feedback(K*G, 1);
    % Calculate step response info
    info = stepinfo(ClosedLoop);
    Ts = info.SettlingTime;
    Mp = info.Overshoot;
    % Calculate cost function J
    J = w1 * Ts + w2 * Mp; 
    J_history(i) = J;
    % Check for convergence
    if i > 1 && abs(J_history (i) - J_history (i-1)) < tolerance 
        fprintf('Converged at iteration %d\n', i);
        J_history = J_history(1:i); % Trim unused portion
        break;
    end
    
    % Check for big divergence
    if i > 2 && ((J_history(i) - J_history(i-1)) > divg_limit) && (J_history(i-1) < J_history(i-2)) 
        fprintf('Diverged at iteration %d\n', i);
        J_history = J_history (1:i); % Trim unused portion
        break;
    end
    
    %Display iteration results
    fprintf('Iteration %d: Kp=%.3f, Ki=%.3f, Kd=%.3f, J=%.3f, Ts=%.3f, Mp=%.3f\n', ...
    i, Kp, Ki, Kd, J, Ts, Mp);
    
    delta = 1e-4;
    
    % Gradient w.r.t Kp Kp_gradient (w1
    Kp_gradient = (w1 * stepinfo(feedback(pid(Kp + delta, Ki, Kd)*G, 1)).SettlingTime ...
                   + w2 * stepinfo(feedback(pid(Kp + delta, Ki, Kd)*G, 1)).Overshoot - J) / delta;

    Ki_gradient = (w1 * stepinfo(feedback(pid(Kp, Ki + delta, Kd)*G, 1)).SettlingTime ...
                   + w2 * stepinfo(feedback(pid(Kp, Ki + delta, Kd)*G, 1)).Overshoot - J) / delta;

    Kd_gradient = (w1 * stepinfo(feedback(pid(Kp, Ki, Kd + delta)*G, 1)).SettlingTime ...
                   + w2 * stepinfo(feedback(pid(Kp, Ki, Kd + delta)*G, 1)).Overshoot - J) / delta;
    
    % Update PID gains using gradient descent and avoid negative/NaN values 
    Kp = max(0, Kp - alpha * Kp_gradient);
    Ki = max(0, Ki - alpha * Ki_gradient);
    Kd = max(0, Kd - alpha * Kd_gradient);
    % If any parameter becomes NaN, reset it to a small positive value 
    if isnan (Kp) || isnan (Ki) || isnan (Kd) || isnan (Kd)
        Kp = max(1e-3, Kp);
        Ki= max(1e-3, Ki); 
        Kd = max(1e-3, Kd);
    end
end

plot(J_history);

%% plot 