function y = measurement_step1(x, C)
    % Range measurements from vehicle position to each beacon
    p = x(1);

    nB = length(C.beacons);
    y = zeros(nB, 1);

    for i = 1:nB
        true_range = abs(p - C.beacons(i));
        noise = C.sigma_range * randn;
        y(i) = true_range + noise;
    end
end
