function dx = NLPlatoonFull(t, x, v0L1, v0L2, v01, v02, dh, k)
% dynamics for a nonlinear platoon model

dx = zeros(12,1);

dx(1) = x(2);
dx(2) = -x(2) + v0L1 + u1(t);
dx(3) = x(4);
dx(4) = -x(4) + v01 + g_fn(x(1) - x(3), dh, k);
dx(5) = x(6);
dx(6) = -x(6) + v02 + g_fn(x(3) - x(5), dh, k);
dx(7) = x(8);
dx(8) = -x(8) + v0L2 + g_fn(x(5) - x(7), dh, k) + u2(t);
dx(9) = x(10);
dx(10) = -x(10) + v01 + g_fn(x(7) - x(9), dh, k);
dx(11) = x(12);
dx(12) = -x(12) + v02 + g_fn(x(9) - x(11), dh, k);

end