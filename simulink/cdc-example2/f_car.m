function dx = f_car(x, u)
%%% Description %%%
%
% Dubin's Car Model
%
% States are [x; y; theta] 
%
% Inputs are [omega; v]

dx = zeros(3,1);
dx(1) = u(2)*cos(x(3));
dx(2) = u(2)*sin(x(3));
dx(3) = u(1);

end
