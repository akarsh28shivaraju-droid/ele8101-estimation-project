function y = measurement_1(x, C)
    % Range measurements from vehicle position to each beacon
    p = x(1);

    nB = length(C.beacons);
    y = zeros(nB, 1);

    for i = 1:nB
        true_range = abs(p - C.beacons(i));
        noise = C.sigma_range * randn;
        y(i) = max(true_range + noise, 0);
    end
end
