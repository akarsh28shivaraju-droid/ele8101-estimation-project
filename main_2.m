clear; clc; close all;
addpath('src');

C = config_2();
data = simulate_2(C);

% Plot circular path
figure;
plot(data.P_true(1,:), data.P_true(2,:), 'LineWidth', 1.5); hold on;
plot(C.beacons(:,1), C.beacons(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
xlabel('x [m]');
ylabel('y [m]');
title('Step 2: True Circular Trajectory and Beacon Locations');
legend('Vehicle Path', 'Beacons');
axis equal;
grid on;

% Plot speed
figure;
plot(data.t, data.X_true(2,:), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Speed [m/s]');
title('Step 2: True Speed');
grid on;

% Plot arc-length
figure;
plot(data.t, data.X_true(1,:), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Arc-length s [m]');
title('Step 2: Distance Travelled Along Circular Path');
grid on;

% Plot measurements
figure;
plot(data.t(2:end), data.Y(1,:), 'LineWidth', 1.0); hold on;
plot(data.t(2:end), data.Y(2,:), 'LineWidth', 1.0);
plot(data.t(2:end), data.Y(3,:), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Measured Range [m]');
title('Step 2: Noisy Beacon Measurements');
legend('Beacon 1', 'Beacon 2', 'Beacon 3');
grid on;
