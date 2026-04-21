clc;
clear;
close all;

%% PARAMETERS

dt = 0.1;
T = 25;
N = T/dt;

sigma_meas = 1.5;      % Given in coursework
sigma_process = 0.2;

true_vx = 10;          % longitudinal velocity
max_vy = 0.55;         % coursework limit

%% 3 BEACONS (outside lane)

b1 = [20 10];
b2 = [120 -10];
b3 = [220 12];

%% TRUE STATES

x_true = zeros(1,N);
vx_true = zeros(1,N);

y_true = zeros(1,N);
vy_true = zeros(1,N);

x_true(1) = 0;
vx_true(1) = true_vx;

%% TRUE MOTION

time = 0:dt:T-dt;

for k = 2:N
    
    % Longitudinal motion
    process_noise = sigma_process * randn;
    vx_true(k) = vx_true(k-1) + process_noise;
    x_true(k) = x_true(k-1) + vx_true(k-1)*dt;
    
    % Bounded lateral motion (important)
    y_true(k) = 1.5 * sin(0.3 * time(k));
    
    % Lateral velocity
    vy_true(k) = (1.5 * 0.3 * cos(0.3 * time(k)));
end

%% MEASUREMENTS FROM 3 BEACONS

z1 = zeros(1,N);
z2 = zeros(1,N);
z3 = zeros(1,N);

for k = 1:N
    
    z1(k) = sqrt((x_true(k)-b1(1))^2 + (y_true(k)-b1(2))^2) ...
            + sigma_meas * randn;
        
    z2(k) = sqrt((x_true(k)-b2(1))^2 + (y_true(k)-b2(2))^2) ...
            + sigma_meas * randn;
        
    z3(k) = sqrt((x_true(k)-b3(1))^2 + (y_true(k)-b3(2))^2) ...
            + sigma_meas * randn;
end

%% EKF INITIALIZATION

x_est = zeros(4,N);

% [x ; vx ; y ; vy]
x_est(:,1) = [0; 10; 0; 0];

P = eye(4);

A = [1 dt 0  0;
     0 1  0  0;
     0 0  1 dt;
     0 0  0 1];

Q = 0.02 * eye(4);
R = sigma_meas^2 * eye(3);

%% EKF LOOP

for k = 2:N
    
    %% Prediction
    
    x_pred = A * x_est(:,k-1);
    P_pred = A * P * A' + Q;
    
    xp = x_pred(1);
    yp = x_pred(3);
    
    %% Measurement Prediction
    
    h1 = sqrt((xp-b1(1))^2 + (yp-b1(2))^2);
    h2 = sqrt((xp-b2(1))^2 + (yp-b2(2))^2);
    h3 = sqrt((xp-b3(1))^2 + (yp-b3(2))^2);
    
    h = [h1; h2; h3];
    
    %% Jacobian Matrix
    
    H = [
        (xp-b1(1))/h1   0   (yp-b1(2))/h1   0;
        (xp-b2(1))/h2   0   (yp-b2(2))/h2   0;
        (xp-b3(1))/h3   0   (yp-b3(2))/h3   0
        ];
    
    %% Measurement Update
    
    z = [z1(k); z2(k); z3(k)];
    
    K = P_pred * H' / (H * P_pred * H' + R);
    
    x_est(:,k) = x_pred + K * (z - h);
    
    %% LATERAL BOUNDARY CONSTRAINT
    
    if x_est(3,k) > 2
        x_est(3,k) = 2;
    elseif x_est(3,k) < -2
        x_est(3,k) = -2;
    end
    
    P = (eye(4) - K * H) * P_pred;
end

%% PLOTS

figure;
plot(x_true, y_true, 'b', 'LineWidth', 2);
hold on;
plot(x_est(1,:), x_est(3,:), 'r--', 'LineWidth', 2);

plot(b1(1), b1(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
plot(b2(1), b2(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
plot(b3(1), b3(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('True Path vs Estimated Path (2D Motion)');
legend('True Path', 'Estimated Path', 'Beacons');
grid on;

figure;
plot(time, y_true, 'b', 'LineWidth', 2);
hold on;
plot(time, x_est(3,:), 'r--', 'LineWidth', 2);

xlabel('Time (s)');
ylabel('Lateral Position (m)');
title('True vs Estimated Lateral Position');
legend('True', 'Estimated');
grid on;

%% ERROR

position_error = mean(sqrt((x_true - x_est(1,:)).^2 + ...
                           (y_true - x_est(3,:)).^2));

fprintf('Average 2D Position Error = %.2f m\n', position_error);