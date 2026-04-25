clear; clc; close all;
addpath('src');

C = config_3();
data = simulate_3(C);

t = data.t;
x_long = data.X_true(1,:);
vx = data.X_true(2,:);
y_lat = data.X_true(3,:);
vy = data.X_true(4,:);

figure;
plot(x_long, y_lat, 'LineWidth', 1.8); hold on;
yline(C.y_max, '--r', 'Upper Bound', 'LineWidth', 1.2);
yline(C.y_min, '--r', 'Lower Bound', 'LineWidth', 1.2);
plot(C.beacons(:,1), C.beacons(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
xlabel('Longitudinal Position x [m]');
ylabel('Lateral Position y [m]');
title('Step 3: Vehicle Path on Straight Road');
legend('Vehicle Path', 'Upper Bound', 'Lower Bound', 'Beacons');
grid on;

figure;
plot(t, y_lat, 'LineWidth', 1.8); hold on;
yline(C.y_max, '--r', 'Upper Bound', 'LineWidth', 1.2);
yline(C.y_min, '--r', 'Lower Bound', 'LineWidth', 1.2);
xlabel('Time [s]');
ylabel('Lateral Position y [m]');
title('Step 3: Lateral Position with Road Bounds');
legend('Lateral Position', 'Upper Bound', 'Lower Bound');
grid on;

figure;
plot(t, vx, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Longitudinal Speed v_x [m/s]');
title('Step 3: Longitudinal Speed');
grid on;

figure;
plot(t, vy, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Lateral Speed v_y [m/s]');
title('Step 3: Lateral Speed');
grid on;

figure;
plot(t(2:end), data.Y(1,:), 'LineWidth', 1.0); hold on;
plot(t(2:end), data.Y(2,:), 'LineWidth', 1.0);
plot(t(2:end), data.Y(3,:), 'LineWidth', 1.0);
plot(t(2:end), data.Y(4,:), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Measured Range [m]');
title('Step 3: Noisy Beacon Measurements');
legend('Beacon 1', 'Beacon 2', 'Beacon 3', 'Beacon 4');
grid on;

run_ekf_3();
