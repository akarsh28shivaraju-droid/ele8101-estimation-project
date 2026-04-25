function est = ekf_4(data, C)
    N = C.N_steps;
    n = 4;
    m = size(C.beacons, 1);

    x_hat = [0; 0; 9; 0];
    P = diag([25, 1, 4, 0.25]);

    Q = diag([1e-3, 1e-3, C.qvs^2, C.qve^2]);
    R = (C.sigma_range^2) * eye(m);

    X_hat = zeros(n, N + 1);
    P_store = zeros(n, n, N + 1);

    X_hat(:, 1) = x_hat;
    P_store(:, :, 1) = P;

    for k = 1:N
        s = x_hat(1);
        e = x_hat(2);
        vs = x_hat(3);
        ve = x_hat(4);

        x_pred = [
            s + C.Ts * vs;
            e + C.Ts * ve;
            vs;
            C.a_lat * ve - C.b_lat * e
        ];

        x_pred(2) = min(max(x_pred(2), C.e_min), C.e_max);
        x_pred(3) = min(max(x_pred(3), C.vs_min), C.vs_max);
        x_pred(4) = min(max(x_pred(4), -C.ve_max), C.ve_max);

        F = [
            1, 0, C.Ts, 0;
            0, 1, 0, C.Ts;
            0, 0, 1, 0;
            0, -C.b_lat, 0, C.a_lat
        ];

        P_pred = F * P * F' + Q;

        y_pred = h_measure_4(x_pred, C);

        H = zeros(m, n);
        step = 1e-4;

        for j = 1:n
            dx = zeros(n, 1);
            dx(j) = step;

            y_plus = h_measure_4(x_pred + dx, C);
            y_minus = h_measure_4(x_pred - dx, C);

            H(:, j) = (y_plus - y_minus) / (2 * step);
        end

        y_meas = data.Y(:, k);
        innovation = y_meas - y_pred;

        S = H * P_pred * H' + R;
        K = P_pred * H' / S;

        x_hat = x_pred + K * innovation;
        P = (eye(n) - K * H) * P_pred;

        x_hat(2) = min(max(x_hat(2), C.e_min), C.e_max);
        x_hat(3) = min(max(x_hat(3), C.vs_min), C.vs_max);
        x_hat(4) = min(max(x_hat(4), -C.ve_max), C.ve_max);

        X_hat(:, k+1) = x_hat;
        P_store(:, :, k+1) = P;
    end

    est.X_hat = X_hat;
    est.P = P_store;
end
    








   

    
    
