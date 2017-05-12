function dx = NLPlatoonFull(t, x, p)
% dynamics for the nonlinear platoon model

dx = zeros(12,1);

dx(1) = x(2);
dx(2) = -x(2) + p.v0L1 + u1(t);
dx(3) = x(4);
dx(4) = -x(4) + p.v01 + g_fn(x(1) - x(3), p.dh, p.k);
dx(5) = x(6);
dx(6) = -x(6) + p.v02 + g_fn(x(3) - x(5), p.dh, p.k);
dx(7) = x(8);
dx(8) = -x(8) + p.v0L2 + g_fn(x(5) - x(7), p.dh, p.k) + u2(t,x(5) - x(7), p);
dx(9) = x(10);
dx(10) = -x(10) + p.v03 + g_fn(x(7) - x(9), p.dh, p.k);
dx(11) = x(12);
dx(12) = -x(12) + p.v04 + g_fn(x(9) - x(11), p.dh, p.k);

end