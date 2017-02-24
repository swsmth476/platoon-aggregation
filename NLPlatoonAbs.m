function dx = NLPlatoonAbs(t,x,p)
% dynamics for the nonlinear platoon abstraction model

dx = zeros(4,1);

dx(1) = x(2);
dx(2) = -x(2) + p.v0L1 + u1(t);
dx(3) = x(4);
dx(4) = -x(4) + p.v0L2 + u2(t);

end