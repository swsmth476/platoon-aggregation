function dx = f_bicycle(x, u)
%%% Description %%%
%
% Dubin's Car Model
%
% States are [x; y; theta; v; omega] 
%
% Inputs are [u_1; u_2]
%

dx = zeros(5,1);
dx(1) = x(4)*cos(x(3));
dx(2) = x(4)*sin(x(3));
dx(3) = x(5);
dx(4) = u(1);
dx(5) = u(2);

end
