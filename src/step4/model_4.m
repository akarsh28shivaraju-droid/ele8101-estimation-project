function x_next = model_4(x, C)
    s = x(1);
    e = x(2);
    vs = x(3);
    ve = x(4);

    w_vs = C.qvs * randn;
    w_ve = C.qve * randn;

    s_next = s + C.Ts * vs;
    e_next = e + C.Ts * ve;

    vs_next = vs + w_vs;
    ve_next = C.a_lat * ve - C.b_lat * e + w_ve;

    vs_next = min(max(vs_next, C.vs_min), C.vs_max);
    ve_next = min(max(ve_next, -C.ve_max), C.ve_max);

    if e_next > C.e_max
        e_next = C.e_max;
        ve_next = min(ve_next, 0);
    elseif e_next < C.e_min
        e_next = C.e_min;
        ve_next = max(ve_next, 0);
    end

    x_next = [s_next; e_next; vs_next; ve_next];
end
