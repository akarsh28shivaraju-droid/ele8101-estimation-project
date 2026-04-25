function y = measurement_4(x, C)
    pos = vehicle_position_4(x, C);

    px = pos(1);
    py = pos(2);

    nB = size(C.beacons, 1);
    y = zeros(nB, 1);

    for i = 1:nB
        bx = C.beacons(i, 1);
        by = C.beacons(i, 2);

        true_range = sqrt((px - bx)^2 + (py - by)^2);
        y(i) = max(true_range + C.sigma_range * randn, 0);
    end
end
