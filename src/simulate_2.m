function data = simulate_2(C)
    X_true = zeros(2, C.N_steps + 1);     % [s; v]
    P_true = zeros(2, C.N_steps + 1);     % [x; y]
    Y = zeros(size(C.beacons,1), C.N_steps);

    X_true(:,1) = [C.s0; C.v0];
    P_true(:,1) = path_2(C.s0, C);

    for k = 1:C.N_steps
        X_true(:,k+1) = model_2(X_true(:,k), C);
        P_true(:,k+1) = path_2(X_true(1,k+1), C);
        Y(:,k) = measurement_2(X_true(:,k+1), C);
    end

    data.X_true = X_true;     % [s; v]
    data.P_true = P_true;     % [x; y]
    data.Y = Y;
    data.t = 0:C.Ts:C.N_steps*C.Ts;
end
