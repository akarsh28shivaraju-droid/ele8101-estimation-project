function x_next = model_1(x, C)

% state is [position; velocity]

p = x(1);
v = x(2);

wv = C.qv * randn;

p_next = p + C.Ts * v;
v_next = v + wv;

x_next = [p_next; v_next];

end
