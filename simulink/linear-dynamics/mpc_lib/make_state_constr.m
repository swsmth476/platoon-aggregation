function [Hx_bar, hx_bar] = make_state_constr(T, Hx, hx)
%%% Summary %%%
% Create state constraints for model predictive controller (MPC)

%%% Description %%%
% For an optimization problem with decision variable
% X(t) = [x(t); x(t+1); ... x(t+T-1)]
%
% make_input_constr returns block diagonal Hx_bar, hx_bar s.t. requiring
% Hx_bar*X(t) <= hx_bar
% is equivalent to requiring 
% Hx*x(t) <= hx, Hx*x(t+1) <= hx, ..., Hx*x(t+T-1) <= hx

Hx_bar = [];
hx_bar = [];

for i = 1:T
    Hx_bar = blkdiag(Hx_bar, Hx);
    hx_bar = [hx_bar; hx];
end

end
