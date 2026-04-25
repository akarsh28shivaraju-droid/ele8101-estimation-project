function C = config_2()
    C.Ts = 0.01;
    C.sigma_range = 1.5;
    C.s0 = 0.0;
    C.v0 = 10.0;
    C.qv = 0.08;
    C.N_steps = 2500;

    % Step 2: simple circular path
    C.Rc = 48.0;

    % 2D beacons outside the circular path
    % each row = [x, y]
    C.beacons = [
        -70,   0;
         70,   0;
          0,  75
    ];
end
