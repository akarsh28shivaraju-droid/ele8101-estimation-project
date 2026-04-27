function pos = vehicle_position_4(x, C)
    s = x(1);
    e = x(2);

    [centre, normal] = path_4_normal(s, C);

    pos = centre + e * normal;
end
