function value = phi(k, s)
% returns the value of the nonlinear interconnection phi, where

% phi(s) = { k*\sqrt(s),    s >= 0
%          { -k*\sqrt(-s),  s < 0

if (s > 0)
    value = k*sqrt(s);
else
    value = -k*sqrt(-s);

end