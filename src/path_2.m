function pos = path_2(s, C)
    theta = mod(s / C.Rc, 2*pi);

    x = C.Rc * cos(theta);
    y = C.Rc * sin(theta);

    pos = [x; y];
end
