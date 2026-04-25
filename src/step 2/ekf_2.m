function est = ekf_2(data, C)
    % EKF for Step 2
    % State: x = [s; v]
    % s = distance travelled along circular path
    % v = longitudinal speed

    N = C.N_steps;
    n = 2;
    m = size(C.beacons, 1);

    % Initial estimate
    x_hat = [0; 9.0];
    P = diag([25, 4]);

    % Process noise covariance
    Q = [1e-4, 0;
         0,    C.qv^2];

    % Measurement noise covariance
    R = (C.sigma_range^2) * eye(m);

    % Linearised state matrix for prediction
    F = [1, C.Ts;
         0, 1];

    % Storage
    X_hat = zeros(n, N+1);
    P_store = zeros(n, n, N+1);
    Y_pred_store = zeros(m, N);

    X_hat(:,1) = x_hat;
    P_store(:,:,1) = P;

    for k = 1:N
        % Prediction
        s_pred = x_hat(1) + C.Ts * x_hat(2);
        v_pred = max(x_hat(2), 0);
        x_pred = [s_pred; v_pred];

        P_pred = F * P * F' + Q;

        % Predicted measurement
        y_pred = zeros(m,1);
        H = zeros(m,n);

        s_val = x_pred(1);
        theta = mod(s_val / C.Rc, 2*pi);

        x_pos = C.Rc * cos(theta);
        y_pos = C.Rc * sin(theta);

        dx_ds = -sin(theta);
        dy_ds =  cos(theta);

        for i = 1:m
            bx = C.beacons(i,1);
            by = C.beacons(i,2);

            rx = x_pos - bx;
            ry = y_pos - by;

            ri = sqrt(rx^2 + ry^2);
            y_pred(i) = ri;

            if ri < 1e-8
                dyi_ds = 0;
            else
                dyi_ds = (rx * dx_ds + ry * dy_ds) / ri;
            end

            H(i,:) = [dyi_ds, 0];
        end

        % Update
        y_meas = data.Y(:,k);
        innov = y_meas - y_pred;

        S = H * P_pred * H' + R;
        K = P_pred * H' / S;

        x_hat = x_pred + K * innov;
        x_hat(2) = max(x_hat(2), 0);

        P = (eye(n) - K * H) * P_pred;

        X_hat(:,k+1) = x_hat;
        P_store(:,:,k+1) = P;
        Y_pred_store(:,k) = y_pred;
    end

    est.X_hat = X_hat;
    est.P = P_store;
    est.Y_pred = Y_pred_store;
end
