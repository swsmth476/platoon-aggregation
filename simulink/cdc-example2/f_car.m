function dx = f_car(x, u)
%%% Description %%%
%
% Dubin's Car Model
%
% States are [x; y; theta] 
%
% Inputs are [v; omega]

dx = zeros(3,1);
dx(1) = u(1)*cos(x(3));
dx(2) = u(1)*sin(x(3));
dx(3) = u(2);

end
