function run_ekf_4(C)
    data = simulate_4(C);
    est = ekf_4(data, C);
    t = data.t;

    P_est = zeros(2, length(t));

    for k = 1:length(t)
        P_est(:, k) = vehicle_position_4(est.X_hat(:, k), C);
    end

    s_true = data.X_true(1, :);
    e_true = data.X_true(2, :);
    vs_true = data.X_true(3, :);
    ve_true = data.X_true(4, :);

    s_est = est.X_hat(1, :);
    e_est = est.X_hat(2, :);
    vs_est = est.X_hat(3, :);
    ve_est = est.X_hat(4, :);

    pos_error = sqrt(sum((data.P_true - P_est).^2, 1));
    s_error = s_true - s_est;
    e_error = e_true - e_est;
    vs_error = vs_true - vs_est;
    ve_error = ve_true - ve_est;

    sigma_s = squeeze(sqrt(est.P(1,1,:)))';
    sigma_e = squeeze(sqrt(est.P(2,2,:)))';
    sigma_vs = squeeze(sqrt(est.P(3,3,:)))';
    sigma_ve = squeeze(sqrt(est.P(4,4,:)))';

    Rl = C.Rc_left;
    Rr = C.rho;
    d = C.d;

    c = (Rl - Rr) / d;
    h = sqrt(1 - c^2);

    theta_top = atan2(h, c);
    theta_bot = atan2(-h, c);

    L_top = sqrt(d^2 - (Rl - Rr)^2);
    L_right = Rr * abs(theta_top - theta_bot);
    L_bottom = L_top;
    L_left = Rl * (2*pi - abs(theta_top - theta_bot));
    L_total = L_top + L_right + L_bottom + L_left;

    N_plot = 2500;
    s_vals = linspace(0, L_total, N_plot);

    P_upper = zeros(2, N_plot);
    P_lower = zeros(2, N_plot);

    for i = 1:N_plot
        [~, P_upper(:, i), P_lower(:, i)] = path_4_with_bounds(s_vals(i), C);
    end

    all_x = [P_upper(1,:), P_lower(1,:), C.beacons(:,1)'];
    all_y = [P_upper(2,:), P_lower(2,:), C.beacons(:,2)'];

    min_x = min(all_x) - 10;
    max_x = max(all_x) + 10;
    min_y = min(all_y) - 10;
    max_y = max(all_y) + 10;

    fig1 = figure('Name', 'Estimator Validation - 2D Map');
    plot(P_upper(1,:), P_upper(2,:), 'k-', 'LineWidth', 1.2, 'HandleVisibility','off'); hold on;
    plot(P_lower(1,:), P_lower(2,:), 'k-', 'LineWidth', 1.2, 'DisplayName','Track Bounds');
    plot(data.P_true(1, :), data.P_true(2, :), 'b-', 'LineWidth', 2, 'DisplayName', 'True Path');
    plot(P_est(1, :), P_est(2, :), 'r--', 'LineWidth', 1.8, 'DisplayName', 'Estimated Path');
    plot(C.beacons(:, 1), C.beacons(:, 2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.8, 'MarkerFaceColor','r', 'DisplayName', 'Beacons');

    xlabel('x [m]');
    ylabel('y [m]');
    title('True vs. Estimated Vehicle Path with Boundaries');
    legend('Location', 'best');

    axis equal;
    xlim([min_x, max_x]);
    ylim([min_y, max_y]);
    grid on;

    exportgraphics(fig1, '1_2D_Validation_Map.png', 'Resolution', 300);

    fig2 = figure('Name', 'Estimated Path Only');
    plot(P_upper(1,:), P_upper(2,:), 'k-', 'LineWidth', 1.2, 'HandleVisibility','off'); hold on;
    plot(P_lower(1,:), P_lower(2,:), 'k-', 'LineWidth', 1.2, 'DisplayName','Track Bounds');
    plot(P_est(1, :), P_est(2, :), 'r--', 'LineWidth', 1.8, 'DisplayName', 'Estimated Path');
    plot(C.beacons(:, 1), C.beacons(:, 2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.8, 'MarkerFaceColor','r', 'DisplayName', 'Beacons');

    xlabel('x [m]');
    ylabel('y [m]');
    title('Estimated Vehicle Path (True Path Removed)');
    legend('Location', 'best');

    axis equal;
    xlim([min_x, max_x]);
    ylim([min_y, max_y]);
    grid on;

    exportgraphics(fig2, '2_Estimated_Path_Only.png', 'Resolution', 300);

    fig3 = figure('Name', 'Lateral Position Tracking');
    plot(t, e_true, 'b', 'LineWidth', 1.8, 'DisplayName', 'True Lateral Pos (y_t)'); hold on;
    plot(t, e_est, 'r--', 'LineWidth', 1.6, 'DisplayName', 'Estimated Lateral Pos');

    yline(C.e_max, 'k--', 'Upper bound', 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');
    yline(C.e_min, 'k--', 'Lower bound', 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');

    xlabel('Time [s]');
    ylabel('Lateral offset e [m]');
    title('Estimator Validation: Lateral Position Bounds');
    legend('Location', 'best');
    ylim([-2.5 2.5]);
    grid on;

    exportgraphics(fig3, '3_Lateral_Position.png', 'Resolution', 300);

    fig4 = figure('Name', 'Velocity Tracking');

    subplot(2,1,1);
    plot(t, vs_true, 'b', 'LineWidth', 1.8); hold on;
    plot(t, vs_est, 'r--', 'LineWidth', 1.6);
    yline(C.vs_max, 'k--', 'Maximum Speed');

    xlabel('Time [s]');
    ylabel('Speed v_s [m/s]');
    title('Longitudinal Velocity Validation');
    legend('True v_s', 'Estimated v_s', 'Location', 'best');
    grid on;

    subplot(2,1,2);
    plot(t, ve_true, 'b', 'LineWidth', 1.5); hold on;
    plot(t, ve_est, 'r--', 'LineWidth', 1.5);
    yline(C.ve_max, 'k--', 'Upper Bound');
    yline(-C.ve_max, 'k--', 'Lower Bound');

    xlabel('Time [s]');
    ylabel('Speed v_e [m/s]');
    title('Lateral Velocity Validation');
    legend('True v_e', 'Estimated v_e', 'Location', 'best');
    grid on;

    exportgraphics(fig4, '4_Velocity_Validation.png', 'Resolution', 300);

    fig5 = figure('Name', 'Estimator Error and Variance Analysis');

    subplot(2,2,1);
    plot(t, s_error, 'b', 'LineWidth', 1); hold on;
    plot(t, 3*sigma_s, 'r--', 'LineWidth', 1);
    plot(t, -3*sigma_s, 'r--', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Error [m]');
    title('Along-track Error (s) with \pm3\sigma Bounds');
    grid on;

    subplot(2,2,2);
    plot(t, e_error, 'b', 'LineWidth', 1); hold on;
    plot(t, 3*sigma_e, 'r--', 'LineWidth', 1);
    plot(t, -3*sigma_e, 'r--', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Error [m]');
    title('Lateral Error (e) with \pm3\sigma Bounds');
    grid on;

    subplot(2,2,3);
    plot(t, vs_error, 'b', 'LineWidth', 1); hold on;
    plot(t, 3*sigma_vs, 'r--', 'LineWidth', 1);
    plot(t, -3*sigma_vs, 'r--', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Error [m/s]');
    title('Longitudinal Speed Error (v_s) with \pm3\sigma');
    grid on;

    subplot(2,2,4);
    plot(t, ve_error, 'b', 'LineWidth', 1); hold on;
    plot(t, 3*sigma_ve, 'r--', 'LineWidth', 1);
    plot(t, -3*sigma_ve, 'r--', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Error [m/s]');
    title('Lateral Speed Error (v_e) with \pm3\sigma');
    grid on;

    exportgraphics(fig5, '5_Variance_Analysis.png', 'Resolution', 300);

    fprintf('\nStep 4 EKF Results\n');
    fprintf('2D position RMSE        = %.4f m\n', sqrt(mean(pos_error.^2)));
    fprintf('Along-track RMSE        = %.4f m\n', sqrt(mean(s_error.^2)));
    fprintf('Lateral offset RMSE     = %.4f m\n', sqrt(mean(e_error.^2)));
    fprintf('Longitudinal speed RMSE = %.4f m/s\n', sqrt(mean(vs_error.^2)));
    fprintf('Lateral speed RMSE      = %.4f m/s\n\n', sqrt(mean(ve_error.^2)));
end
