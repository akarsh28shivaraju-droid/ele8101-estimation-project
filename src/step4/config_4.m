function C = config_4()
    C.Ts = 0.01;
    C.sigma_range = 1.5;
    C.N_steps = 5000;

    C.R = 50;
    C.r = 46;
    C.rho = 20;
    C.d = 100;
    C.Rc_left = (C.R + C.r) / 2;

    C.road_width = C.R - C.r;
    C.e_min = -2;
    C.e_max = 2;

    C.s0 = 0;
    C.e0 = 0;
    C.vs0 = 10;
    C.ve0 = 0;

    C.qvs = 0.03;
    C.qve = 0.015;

    C.a_lat = 0.990;
    C.b_lat = 0.03;

    C.vs_min = 0;
    C.vs_max = 25;
    C.ve_max = 0.55;

    C.beacons = [
        -60,   0;
         50,  70;
        130,   0
    ];
end
