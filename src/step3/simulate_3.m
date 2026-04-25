function data = simulate_3(C)
    % State: [x_long; vx; y_lat; vy]

    X_true = zeros(4, C.N_steps + 1);
    Y = zeros(size(C.beacons,1), C.N_steps);

    X_true(:,1) = [C.x0; C.vx0; C.y0; C.vy0];

    for k = 1:C.N_steps
        X_true(:,k+1) = model_3(X_true(:,k), C);
        Y(:,k) = measurement_3(X_true(:,k+1), C);
    end

    data.X_true = X_true;
    data.Y = Y;
    data.t = 0:C.Ts:C.N_steps*C.Ts;
end
