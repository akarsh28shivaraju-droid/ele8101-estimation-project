clc;
clear;
close all;

%% PARAMETERS

dt = 0.1;              % Sampling time
T = 20;                % Total simulation time
N = T/dt;              % Number of steps

true_velocity = 10;    % Approx constant velocity (m/s)
sigma_process = 0.2;   % Process noise std
sigma_meas = 1.5;      % Measurement noise std

beacon = 50;           % Beacon position (1D)

%% TRUE STATES

x_true = zeros(1,N);
v_true = zeros(1,N);

x_true(1) = 0;
v_true(1) = true_velocity;

for k = 2:N
    process_noise = sigma_process * randn;
    
    v_true(k) = v_true(k-1) + process_noise;
    x_true(k) = x_true(k-1) + v_true(k-1)*dt;
end

%% MEASUREMENTS

z = zeros(1,N);

for k = 1:N
    measurement_noise = sigma_meas * randn;
    z(k) = abs(x_true(k) - beacon) + measurement_noise;
end

%% EKF INITIALIZATION

x_est = zeros(2,N);    % [position; velocity]

x_est(:,1) = [5; 8];   % Initial guess

P = eye(2);            % Initial covariance

A = [1 dt;
     0 1];

Q = [0.1 0;
     0 0.1];

R = sigma_meas^2;

%% EKF LOOP

for k = 2:N
    
    %% Prediction Step
    
    x_pred = A * x_est(:,k-1);
    P_pred = A * P * A' + Q;
    
    %% Measurement Model
    
    x_position = x_pred(1);
    
    h = abs(x_position - beacon);
    
    if x_position >= beacon
        H = [1 0];
    else
        H = [-1 0];
    end
    
    %% Update Step
    
    K = P_pred * H' / (H * P_pred * H' + R);
    
    x_est(:,k) = x_pred + K * (z(k) - h);
    
    P = (eye(2) - K * H) * P_pred;
end

%% PLOTS

time = 0:dt:T-dt;

figure;
plot(time, x_true, 'b', 'LineWidth', 2);
hold on;
plot(time, x_est(1,:), 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (m)');
title('True Position vs Estimated Position');
legend('True Position', 'Estimated Position');
grid on;

figure;
plot(time, v_true, 'b', 'LineWidth', 2);
hold on;
plot(time, x_est(2,:), 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('True Velocity vs Estimated Velocity');
legend('True Velocity', 'Estimated Velocity');
grid on;

%% ERROR CALCULATION

position_error = mean(abs(x_true - x_est(1,:)));
velocity_error = mean(abs(v_true - x_est(2,:)));

fprintf('Average Position Error = %.2f m\n', position_error);
fprintf('Average Velocity Error = %.2f m/s\n', velocity_error);