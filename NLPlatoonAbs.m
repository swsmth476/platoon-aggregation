function dx = NLPlatoonAbs(t,x,v0,dh,k)
% nonlinear dynamics of the platoon abstraction

dx = zeros(4,1);

dx(1) = x(2);
dx(2) = -x(2) + v0;
dx(3) = x(4);
dx(4) = -x(4) + v0 + g_fn(x(1) - x(3),dh,k);

end