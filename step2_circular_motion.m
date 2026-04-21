clc;
clear;
close all;

%% PARAMETERS

dt = 0.1;              % Sampling time
T = 20;                % Total simulation time
N = T/dt;

R = 50;                % Circular track radius (m)

true_speed = 10;       % Approx speed (m/s)
true_omega = true_speed / R;   % Angular velocity

sigma_process = 0.01;  % Process noise std
sigma_meas = 1.5;      % Measurement noise std

%% BEACON POSITION (outside track)

bx = 70;
by = 20;

%% TRUE STATES

theta_true = zeros(1,N);
omega_true = zeros(1,N);

theta_true(1) = 0;
omega_true(1) = true_omega;

for k = 2:N
    
    process_noise = sigma_process * randn;
    
    omega_true(k) = omega_true(k-1) + process_noise;
    theta_true(k) = theta_true(k-1) + omega_true(k-1)*dt;
end

%% TRUE POSITION (x,y)

x_true = R * cos(theta_true);
y_true = R * sin(theta_true);

%% MEASUREMENTS (distance to beacon)

z = zeros(1,N);

for k = 1:N
    
    distance_true = sqrt((x_true(k)-bx)^2 + (y_true(k)-by)^2);
    measurement_noise = sigma_meas * randn;
    
    z(k) = distance_true + measurement_noise;
end

%% EKF INITIALIZATION

x_est = zeros(2,N);   % [theta; omega]

x_est(:,1) = [0.05; true_omega];

P = eye(2);

A = [1 dt;
     0 1];

Q = [0.01 0;
     0 0.01];

R_meas = sigma_meas^2;

%% EKF LOOP

for k = 2:N
    
    %% Prediction
    
    x_pred = A * x_est(:,k-1);
    P_pred = A * P * A' + Q;
    
    theta = x_pred(1);
    
    %% Predicted position
    
    x_pred_pos = R * cos(theta);
    y_pred_pos = R * sin(theta);
    
    %% Measurement model
    
    h = sqrt((x_pred_pos - bx)^2 + (y_pred_pos - by)^2);
    
    %% Jacobian H
    
    dh_dtheta = ((bx - x_pred_pos)*(R*sin(theta)) + ...
             (by - y_pred_pos)*(-R*cos(theta))) / h;

H = [dh_dtheta 0];    
    %% Update
    
    K = P_pred * H' / (H * P_pred * H' + R_meas);
    
    x_est(:,k) = x_pred + K * (z(k) - h);
    
    P = (eye(2) - K * H) * P_pred;
end

%% ESTIMATED POSITION

theta_est = x_est(1,:);

x_est_plot = R * cos(theta_est);
y_est_plot = R * sin(theta_est);

%% PLOTS

figure;
plot(x_true, y_true, 'b', 'LineWidth', 2);
hold on;
plot(x_est_plot, y_est_plot, 'r--', 'LineWidth', 2);
plot(bx, by, 'ko', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('True Circular Path vs Estimated Path');
legend('True Path', 'Estimated Path', 'Beacon');
grid on;
axis equal;

figure;
time = 0:dt:T-dt;

plot(time, theta_true, 'b', 'LineWidth', 2);
hold on;
plot(time, theta_est, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Angular Position (rad)');
title('True vs Estimated Angular Position');
legend('True', 'Estimated');
grid on;

%% ERROR

position_error = mean(sqrt((x_true - x_est_plot).^2 + ...
                           (y_true - y_est_plot).^2));

fprintf('Average Circular Position Error = %.2f m\n', position_error);