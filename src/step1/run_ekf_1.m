function run_ekf_1()

C = config_1();
data = simulate_1(C);
est = ekf_1(data, C);

t = data.t;

figure;
plot(t, data.X_true(1, :), 'LineWidth', 1.5); hold on;
plot(t, est.X_hat(1, :), '--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Position [m]');
title('True and Estimated Position');
legend('True Position', 'Estimated Position');
grid on;

figure;
plot(t, data.X_true(2, :), 'LineWidth', 1.5); hold on;
plot(t, est.X_hat(2, :), '--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('True and Estimated Velocity');
legend('True Velocity', 'Estimated Velocity');
grid on;

pos_error = data.X_true(1, :) - est.X_hat(1, :);
vel_error = data.X_true(2, :) - est.X_hat(2, :);

figure;
plot(t, pos_error, 'LineWidth', 1.5); hold on;
plot(t, vel_error, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Error');
title('Estimation Errors');
legend('Position Error', 'Velocity Error');
grid on;

pos_rmse = sqrt(mean(pos_error.^2));
vel_rmse = sqrt(mean(vel_error.^2));

fprintf('Position RMSE = %.4f m\n', pos_rmse);
fprintf('Velocity RMSE = %.4f m/s\n', vel_rmse);

end
