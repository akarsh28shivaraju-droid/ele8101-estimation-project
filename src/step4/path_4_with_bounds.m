function [centre, upper, lower] = path_4_with_bounds(s, C)
    [centre, normal] = path_4_normal(s, C);
    half_width = C.road_width / 2;

    upper = centre + half_width * normal;
    lower = centre - half_width * normal;
end
