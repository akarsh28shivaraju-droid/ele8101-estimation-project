function [centre, normal] = path_4_normal(s, C)
    ds = 0.01;

    centre = path_4(s, C);
    p1 = path_4(s - ds, C);
    p2 = path_4(s + ds, C);

    tangent = p2 - p1;
    tangent = tangent / norm(tangent);

    normal = [-tangent(2); tangent(1)];
end
