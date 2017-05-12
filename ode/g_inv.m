function d = g_inv(y, dh, k)
% inverse map of g(x) = y
% (see g_fn.m)

d = dh*exp(y/k);

end