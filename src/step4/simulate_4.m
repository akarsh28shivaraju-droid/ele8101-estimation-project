function data = simulate_4(C)
    X_true = zeros(4, C.N_steps + 1);
    P_true = zeros(2, C.N_steps + 1);
    Y = zeros(size(C.beacons, 1), C.N_steps);

    X_true(:, 1) = [C.s0; C.e0; C.vs0; C.ve0];
    P_true(:, 1) = vehicle_position_4(X_true(:, 1), C);

    for k = 1:C.N_steps
        X_true(:, k+1) = model_4(X_true(:, k), C);
        P_true(:, k+1) = vehicle_position_4(X_true(:, k+1), C);
        Y(:, k) = measurement_4(X_true(:, k+1), C);
    end

    data.X_true = X_true;
    data.P_true = P_true;
    data.Y = Y;
    data.t = 0:C.Ts:C.N_steps*C.Ts;
end
