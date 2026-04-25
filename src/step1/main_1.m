clear; clc; close all;
addpath('src');

C = config_1();
data = simulate_1(C);

figure;
plot(data.t, data.X_true(1, :), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Position [m]');
title('True Position');
grid on;

figure;
plot(data.t, data.X_true(2, :), 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('True Velocity');
grid on;

figure;
plot(data.t(2:end), data.Y(1, :), 'LineWidth', 1.0); hold on;
plot(data.t(2:end), data.Y(2, :), 'LineWidth', 1.0);
xlabel('Time [s]');
ylabel('Measured Range [m]');
title('Noisy Beacon Measurements');
legend('Beacon 1', 'Beacon 2');
grid on;

run_ekf_1();
