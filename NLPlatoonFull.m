function dx = NLPlatoonFull(t, x, v0, v01, v02)
% dynamics for a nonlinear platoon model

dx = zeros(12,1);

dx(1) = x(2);
dx(2) = -x(2) + v0;
dx(3) = x(4);
dx(4) = -x(4) + v01 + g_fn(x(1) - x(3));
dx(5) = x(6);
dx(6) = -x(6) + v02 + g_fn(x(3) - x(5));
dx(7) = x(8);
dx(8) = -x(8) + v0 + g_fn(x(5) - x(7));
dx(9) = x(10);
dx(10) = -x(10) + v01 + g_fn(x(7) - x(9));
dx(11) = x(12);
dx(12) = -x(12) + v02 + g_fn(x(9) - x(11));

end