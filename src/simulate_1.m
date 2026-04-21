function data = simulate_step1(C)
    % State: x = [position; velocity]

    X_true = zeros(2, C.N_steps + 1);
    Y = zeros(length(C.beacons), C.N_steps);

    X_true(:,1) = [C.p0; C.v0];

    for k = 1:C.N_steps
        X_true(:,k+1) = model_step1(X_true(:,k), C);
        Y(:,k) = measurement_step1(X_true(:,k+1), C);
    end

    data.X_true = X_true;
    data.Y = Y;
    data.t = 0:C.Ts:C.N_steps*C.Ts;
end
