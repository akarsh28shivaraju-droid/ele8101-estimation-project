function y = measurement_3(x, C)
    px = x(1);
    py = x(3);

    nB = size(C.beacons, 1);
    y = zeros(nB, 1);

    for i = 1:nB
        bx = C.beacons(i,1);
        by = C.beacons(i,2);

        true_range = sqrt((px - bx)^2 + (py - by)^2);
        noise = C.sigma_range * randn;

        y(i) = max(true_range + noise, 0);
    end
end
