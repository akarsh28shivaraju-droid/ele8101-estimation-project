function run_ekf_3()
    C = config_3();
    data = simulate_3(C);
    est = ekf_3(data, C);

    t = data.t;

    x_true  = data.X_true(1,:);
    vx_true = data.X_true(2,:);
    y_true  = data.X_true(3,:);
    vy_true = data.X_true(4,:);

    x_est  = est.X_hat(1,:);
    vx_est = est.X_hat(2,:);
    y_est  = est.X_hat(3,:);
    vy_est = est.X_hat(4,:);

    figure;
    plot(x_true, y_true, 'LineWidth', 1.8); hold on;
    plot(x_est, y_est, '--', 'LineWidth', 1.8);
    yline(C.y_max, '--r', 'Upper Bound', 'LineWidth', 1.2);
    yline(C.y_min, '--r', 'Lower Bound', 'LineWidth', 1.2);
    plot(C.beacons(:,1), C.beacons(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
    xlabel('Longitudinal Position x [m]');
    ylabel('Lateral Position y [m]');
    title('Step 3 EKF: True vs Estimated Vehicle Path');
    legend('True Path', 'Estimated Path', 'Upper Bound', 'Lower Bound', 'Beacons');
    grid on;

    figure;
    plot(t, x_true, 'LineWidth', 1.5); hold on;
    plot(t, x_est, '--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Longitudinal Position x [m]');
    title('Step 3 EKF: True vs Estimated Longitudinal Position');
    legend('True x', 'Estimated x');
    grid on;

    figure;
    plot(t, vx_true, 'LineWidth', 1.5); hold on;
    plot(t, vx_est, '--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Longitudinal Speed v_x [m/s]');
    title('Step 3 EKF: True vs Estimated Longitudinal Speed');
    legend('True v_x', 'Estimated v_x');
    grid on;

    figure;
    plot(t, y_true, 'LineWidth', 1.5); hold on;
    plot(t, y_est, '--', 'LineWidth', 1.5);
    yline(C.y_max, '--r', 'Upper Bound', 'LineWidth', 1.2);
    yline(C.y_min, '--r', 'Lower Bound', 'LineWidth', 1.2);
    xlabel('Time [s]');
    ylabel('Lateral Position y [m]');
    title('Step 3 EKF: True vs Estimated Lateral Position');
    legend('True y', 'Estimated y', 'Upper Bound', 'Lower Bound');
    grid on;

    figure;
    plot(t, vy_true, 'LineWidth', 1.5); hold on;
    plot(t, vy_est, '--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Lateral Speed v_y [m/s]');
    title('Step 3 EKF: True vs Estimated Lateral Speed');
    legend('True v_y', 'Estimated v_y');
    grid on;

    ex  = x_true  - x_est;
    evx = vx_true - vx_est;
    ey  = y_true  - y_est;
    evy = vy_true - vy_est;

    figure;
    plot(t, ex, 'LineWidth', 1.2); hold on;
    plot(t, ey, 'LineWidth', 1.2);
    xlabel('Time [s]');
    ylabel('Error');
    title('Step 3 EKF: Position Errors');
    legend('x Error', 'y Error');
    grid on;

    figure;
    plot(t, evx, 'LineWidth', 1.2); hold on;
    plot(t, evy, 'LineWidth', 1.2);
    xlabel('Time [s]');
    ylabel('Error');
    title('Step 3 EKF: Velocity Errors');
    legend('v_x Error', 'v_y Error');
    grid on;

    x_rmse  = sqrt(mean(ex.^2));
    vx_rmse = sqrt(mean(evx.^2));
    y_rmse  = sqrt(mean(ey.^2));
    vy_rmse = sqrt(mean(evy.^2));

    fprintf('Step 3 Longitudinal Position RMSE = %.4f m\n', x_rmse);
    fprintf('Step 3 Longitudinal Speed RMSE    = %.4f m/s\n', vx_rmse);
    fprintf('Step 3 Lateral Position RMSE      = %.4f m\n', y_rmse);
    fprintf('Step 3 Lateral Speed RMSE         = %.4f m/s\n', vy_rmse);
end
