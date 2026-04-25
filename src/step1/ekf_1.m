function est = ekf_1(data, C)

N = C.N_steps;
n = 2;
m = length(C.beacons);

x_hat = [0; 8];
P = diag([25, 4]);

Q = [0, 0;
     0, C.qv^2];

R = (C.sigma_range^2) * eye(m);

F = [1, C.Ts;
     0, 1];

X_hat = zeros(n, N + 1);
P_store = zeros(n, n, N + 1);

X_hat(:, 1) = x_hat;
P_store(:, :, 1) = P;

for k = 1:N

    % prediction
    x_pred = [
        x_hat(1) + C.Ts * x_hat(2);
        x_hat(2)
    ];

    P_pred = F * P * F' + Q;

    % predicted range measurements
    y_pred = zeros(m, 1);
    H = zeros(m, n);

    for i = 1:m

        beacon = C.beacons(i);
        d = x_pred(1) - beacon;

        y_pred(i) = abs(d);

        if d > 0
            H(i, :) = [1, 0];
        elseif d < 0
            H(i, :) = [-1, 0];
        else
            H(i, :) = [0, 0];
        end

    end

    % correction
    y_meas = data.Y(:, k);
    innovation = y_meas - y_pred;

    S = H * P_pred * H' + R;
    K = P_pred * H' / S;

    x_hat = x_pred + K * innovation;
    P = (eye(n) - K * H) * P_pred;

    X_hat(:, k+1) = x_hat;
    P_store(:, :, k+1) = P;

end

est.X_hat = X_hat;
est.P = P_store;

end
