function f_approx = f_car_approx(x,u)
%%% Description %%%
%
% 3rd order Taylor approximation of Dubin's car model
%
% States are [x; y; theta] 
%
% Inputs are [omega; v]

f_approx = [u(2)*(1 - 0.47*x(3)^2);
            u(2)*(x(3) - 0.16*x(3)^3);
            u(1)];

end
