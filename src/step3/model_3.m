function x_next = model_3(x, C)
    % State: x = [x_long; vx; y_lat; vy]

    x_long = x(1);
    vx     = x(2);
    y_lat  = x(3);
    vy     = x(4);

    wvx = C.qvx * randn;
    wvy = C.qvy * randn;

    x_long_next = x_long + C.Ts * vx;
    vx_next = max(vx + wvx, 0);

    y_lat_next = y_lat + C.Ts * vy;
    vy_next = C.a_lat * vy - C.b_lat * y_lat + wvy;

    if y_lat_next > C.y_max
        y_lat_next = C.y_max;
        vy_next = min(vy_next, 0);
    elseif y_lat_next < C.y_min
        y_lat_next = C.y_min;
        vy_next = max(vy_next, 0);
    end

    x_next = [x_long_next; vx_next; y_lat_next; vy_next];
end
