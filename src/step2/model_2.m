function x_next = model_2(x, C)
    % x = [arc_length; speed]

    s = x(1);
    v = x(2);

    wv = C.qv * randn;

    s_next = s + C.Ts * v;
    v_next = max(v + wv, 0);

    x_next = [s_next; v_next];
end
