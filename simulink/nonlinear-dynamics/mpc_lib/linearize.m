function [A, B] = linearize(x, gamma, A1, A2, A3, A4, k)
% linearize aggregate tank dynamics about state x

% get A matrix
A = zeros(2);
A(1,1) = -gamma - 1/(A1 + A2)*dphi(k, x(1) - x(2));
A(1,2) = 1/(A1 + A2)*dphi(k, x(1) - x(2));
A(2,1) = 1/(A3 + A4)*dphi(k, x(1) - x(2));
A(2,2) = -gamma - 1/(A3 + A4)*dphi(k, x(1) - x(2));

% get B matrix
B = eye(2);

end