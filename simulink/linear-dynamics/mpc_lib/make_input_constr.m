function [Hu_bar, hu_bar] = make_input_constr(T, Hu, hu)
%%% Summary %%%
% Create input constraints for model predictive controller (MPC)

%%% Description %%%
% For an optimization problem with decision variable
% U(t) = [u(t); u(t+1); ... u(t+T-1)]
%
% make_input_constr returns block diagonal Hu_bar, hu_bar s.t. requiring
% Hu_bar*U(t) <= hu_bar
% is equivalent to requiring 
% Hu*u(t) <= hu, Hu*u(t+1) <= hu, ..., Hu*u(t+T-1) <= hu
%

Hu_bar = [];
hu_bar = [];

for i = 1:T
    Hu_bar = blkdiag(Hu_bar, Hu);
    hu_bar = [hu_bar; hu];
end

end

