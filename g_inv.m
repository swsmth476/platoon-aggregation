function distance = g_inv(y, desired_headway, gain)
% inverse map of g(x) = y
% (see g_fn.m)

distance = desired_headway*exp(y/gain);

end