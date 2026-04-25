function est = ekf_3(data, C)
    % EKF for Step 3
    % State: x = [x_long; vx; y_lat; vy]

    N = C.N_steps;
    n = 4;
    m = size(C.beacons, 1);

    x_hat = [0; 9.0; 0; 0];

    P = diag([25, 4, 2, 1]);

    Q = diag([1e-4, C.qvx^2, 1e-3, C.qvy^2]);

    R = (C.sigma_range^2) * eye(m);

    X_hat = zeros(n, N+1);
    P_store = zeros(n, n, N+1);
    Y_pred_store = zeros(m, N);

    X_hat(:,1) = x_hat;
    P_store(:,:,1) = P;

    for k = 1:N
        x_long = x_hat(1);
        vx     = x_hat(2);
        y_lat  = x_hat(3);
        vy     = x_hat(4);

        x_long_pred = x_long + C.Ts * vx;
        vx_pred     = max(vx, 0);
        y_lat_pred  = y_lat + C.Ts * vy;
        vy_pred     = C.a_lat * vy - C.b_lat * y_lat;

        x_pred = [x_long_pred; vx_pred; y_lat_pred; vy_pred];

        if x_pred(3) > C.y_max
            x_pred(3) = C.y_max;
        elseif x_pred(3) < C.y_min
            x_pred(3) = C.y_min;
        end

        F = [1, C.Ts, 0,       0;
             0, 1,    0,       0;
             0, 0,    1,    C.Ts;
             0, 0, -C.b_lat, C.a_lat];

        P_pred = F * P * F' + Q;

        y_pred = zeros(m,1);
        H = zeros(m,n);

        px = x_pred(1);
        py = x_pred(3);

        for i = 1:m
            bx = C.beacons(i,1);
            by = C.beacons(i,2);

            rx = px - bx;
            ry = py - by;

            ri = sqrt(rx^2 + ry^2);
            y_pred(i) = ri;

            if ri < 1e-8
                H(i,:) = [0, 0, 0, 0];
            else
                H(i,:) = [rx/ri, 0, ry/ri, 0];
            end
        end

        y_meas = data.Y(:,k);
        innov = y_meas - y_pred;

        S = H * P_pred * H' + R;
        K = P_pred * H' / S;

        x_hat = x_pred + K * innov;
        P = (eye(n) - K * H) * P_pred;

        x_hat(2) = max(x_hat(2), 0);

        if x_hat(3) > C.y_max
            x_hat(3) = C.y_max;
        elseif x_hat(3) < C.y_min
            x_hat(3) = C.y_min;
        end

        X_hat(:,k+1) = x_hat;
        P_store(:,:,k+1) = P;
        Y_pred_store(:,k) = y_pred;
    end

    est.X_hat = X_hat;
    est.P = P_store;
    est.Y_pred = Y_pred_store;
end
