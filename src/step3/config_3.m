function C = config_3()
    C.Ts = 0.01;
    C.sigma_range = 1.5;
    C.N_steps = 2500;

    % Initial states
    C.x0 = 0.0;
    C.vx0 = 10.0;
    C.y0 = 0.0;
    C.vy0 = 0.0;

    % Motion noise
    C.qvx = 0.08;
    C.qvy = 0.25;

    % Road geometry
    C.road_width = 4.0;
    C.y_min = -2.0;
    C.y_max =  2.0;

    % Mean-reverting lateral dynamics
    C.a_lat = 0.995;
    C.b_lat = 0.08;

    % Beacons outside road bounds
    C.beacons = [
         20, -6;
         20,  6;
         80, -6;
         80,  6
    ];
end
