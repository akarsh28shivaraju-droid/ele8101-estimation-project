function pos = path_4(s, C)
    Rl = C.Rc_left;
    Rr = C.rho;
    d = C.d;
    A = [0; 0];
    B = [d; 0];

    c = (Rl - Rr) / d;
    h = sqrt(1 - c^2);
    n_top = [c; h];
    n_bot = [c; -h];

    P_LT = A + Rl * n_top;
    P_LB = A + Rl * n_bot;
    P_RT = B + Rr * n_top;
    P_RB = B + Rr * n_bot;

    theta_LT = atan2(P_LT(2), P_LT(1));
    theta_LB = atan2(P_LB(2), P_LB(1));
    theta_RT = atan2(P_RT(2)-B(2), P_RT(1)-B(1));
    theta_RB = atan2(P_RB(2)-B(2), P_RB(1)-B(1));

    L_top = norm(P_RT - P_LT);
    L_right = Rr * abs(theta_RT - theta_RB);
    L_bottom = norm(P_LB - P_RB);
    L_left = Rl * (2*pi - abs(theta_LT - theta_LB));
    L_total = L_top + L_right + L_bottom + L_left;

    s = mod(s, L_total);

    if s < L_top
        tau = s / L_top;
        pos = P_LT + tau * (P_RT - P_LT);
    elseif s < L_top + L_right
        ss = s - L_top;
        frac = ss / L_right;
        theta = theta_RT + frac * (theta_RB - theta_RT);
        pos = B + Rr * [cos(theta); sin(theta)];
    elseif s < L_top + L_right + L_bottom
        ss = s - L_top - L_right;
        tau = ss / L_bottom;
        pos = P_RB + tau * (P_LB - P_RB);
    else
        ss = s - L_top - L_right - L_bottom;
        frac = ss / L_left;
        theta_start = theta_LB;
        theta_end = theta_LT;
        if theta_end > theta_start
            theta_end = theta_end - 2*pi;
        end
        theta = theta_start + frac * (theta_end - theta_start);
        pos = A + Rl * [cos(theta); sin(theta)];
    end
end
