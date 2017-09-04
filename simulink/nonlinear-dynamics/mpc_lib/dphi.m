function value = dphi(k, s)
% derivative of phi function

% ignore the derivative of phi for low values
threshold = .25;

if(abs(s) < threshold)
    value = 0;
elseif(s >= 0)
    value = k/2*(s)^(-1/2);
else
    value = k/2*(-s)^(-1/2);

end