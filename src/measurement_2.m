function y = measurement_2(x, C)
    % x = [arc_length; speed]
    pos = path_2(x(1), C);

    nB = size(C.beacons, 1);
    y = zeros(nB, 1);

    for i = 1:nB
        bx = C.beacons(i, 1);
        by = C.beacons(i, 2);

        true_range = sqrt((pos(1) - bx)^2 + (pos(2) - by)^2);
        noise = C.sigma_range * randn;

        y(i) = max(true_range + noise, 0);
    end
end
