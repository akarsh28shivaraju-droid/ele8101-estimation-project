function C = config_2()
    C.Ts = 0.01;                  % 100 Hz
    C.sigma_range = 1.5;          % range std [m]
    C.s0 = 0.0;                   % initial arc-length [m]
    C.v0 = 10.0;                  % initial speed [m/s]
    C.qv = 0.08;                  % speed process noise std
    C.N_steps = 800;              % simulation steps

    C.Rc = 48.0;                  % circle radius [m]

    % 2D beacons outside the circular path
    % each row = [x, y]
    C.beacons = [
        -70,   0;
         70,   0;
          0,  75
    ];
end
