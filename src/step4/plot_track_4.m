function plot_track_geometry(C)
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

    P_centre = zeros(2, N_plot);
    P_upper = zeros(2, N_plot);
    P_lower = zeros(2, N_plot);

    for k = 1:N_plot
        [P_centre(:, k), P_upper(:, k), P_lower(:, k)] = path_4_with_bounds(s_vals(k), C);
    end

    fig0 = figure('Name', 'Track Initialization');
    plot(P_centre(1, :), P_centre(2, :), 'b', 'LineWidth', 2); hold on;
    plot(P_upper(1, :), P_upper(2, :), 'k', 'LineWidth', 1.5);
    plot(P_lower(1, :), P_lower(2, :), 'k', 'LineWidth', 1.5);
    plot(C.beacons(:, 1), C.beacons(:, 2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.8);

    xlabel('x [m]');
    ylabel('y [m]');
    title('Track Centreline, Lane Bounds and Beacon Locations');
    legend('Centreline', 'Upper Bound', 'Lower Bound', 'Beacons', 'Location', 'northeast');

    axis equal;
    grid on;

    exportgraphics(fig0, '0_Track_Initialization.png', 'Resolution', 300);
end
