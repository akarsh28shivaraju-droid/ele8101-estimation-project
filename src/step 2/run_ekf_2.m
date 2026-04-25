function run_ekf_2()
    C = config_2();
    data = simulate_2(C);
    est = ekf_2(data, C);

    t = data.t;

    P_est = zeros(2, length(t));
    for k = 1:length(t)
        P_est(:,k) = path_2(est.X_hat(1,k), C);
    end

    figure;
    plot(data.P_true(1,:), data.P_true(2,:), 'LineWidth', 1.8); hold on;
    plot(P_est(1,:), P_est(2,:), '--', 'LineWidth', 1.8);
    plot(C.beacons(:,1), C.beacons(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
    xlabel('x [m]');
    ylabel('y [m]');
    title('Step 2 EKF: True vs Estimated Circular Path');
    legend('True Path', 'Estimated Path', 'Beacons');
    axis equal;
    grid on;

    figure;
    plot(t, data.X_true(1,:), 'LineWidth', 1.5); hold on;
    plot(t, est.X_hat(1,:), '--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Arc-length s [m]');
    title('Step 2 EKF: True vs Estimated Arc-length');
    legend('True s', 'Estimated s');
    grid on;

    figure;
    plot(t, data.X_true(2,:), 'LineWidth', 1.5); hold on;
    plot(t, est.X_hat(2,:), '--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Speed [m/s]');
    title('Step 2 EKF: True vs Estimated Speed');
    legend('True Speed', 'Estimated Speed');
    grid on;

    s_error = data.X_true(1,:) - est.X_hat(1,:);
    v_error = data.X_true(2,:) - est.X_hat(2,:);

    figure;
    plot(t, s_error, 'LineWidth', 1.5); hold on;
    plot(t, v_error, 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Error');
    title('Step 2 EKF: Estimation Errors');
    legend('Arc-length Error', 'Speed Error');
    grid on;

    xy_error = data.P_true - P_est;
    pos2d_error = sqrt(sum(xy_error.^2, 1));

    figure;
    plot(t, pos2d_error, 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('2D Position Error [m]');
    title('Step 2 EKF: 2D Position Error');
    grid on;

    s_rmse = sqrt(mean(s_error.^2));
    v_rmse = sqrt(mean(v_error.^2));
    pos2d_rmse = sqrt(mean(pos2d_error.^2));

    fprintf('Step 2 Arc-length RMSE = %.4f m\n', s_rmse);
    fprintf('Step 2 Speed RMSE      = %.4f m/s\n', v_rmse);
    fprintf('Step 2 2D Position RMSE = %.4f m\n', pos2d_rmse);
end
