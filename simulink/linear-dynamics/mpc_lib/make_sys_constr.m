function [G, M] = make_sys_constr(T, A, B, theta, x0)
%%% Summary %%%
% Create system constraints for model predictive controller (MPC)

%%% Description %%%
% System dynamics are of the form:
% x(t+1) = A*x(t) + B*u(t) + theta       (1)
%
% For an optimization problem with decision variables (both col. vectors)
% U(t) = [u(t); u(t+1); ... u(t+T-1)]
% X(t+1) = [x(t+1); x(t+2); ... x(t+T)]
%
% make_sys_constr returns G and M s.t. requiring
% X(t+1) = G*U(t) + M
% is equivalent to requiring that state updates (1) are respected

n = size(x0,1); % state dimension
m = size(B,2); % input dimension

AB = zeros(n, T*m);
M = zeros(n, T);

AB(:,1:m) = B;
M(:,1) = A*x0 + theta;

% store all A^k*B and M(k)
for i = 1:T-1
    idx = (i*m+1):(i*m+m);
    idx_prev = idx - m;
    AB(:,idx) = A*AB(:,idx_prev);
    M(:,i+1) = A*M(:,i) + theta;
end
M = reshape(M, n*T, 1);

G = zeros(n*T, m*T);

for i = 0:T-1
    rows = (i*n+1):(i*n+n);
    cols = 1:m;
    while(rows(1) < n*T)
        G(rows, cols) = AB(:,(i*m+1):(i*m+m));
        rows = rows + n;
        cols = cols + m;
    end
end

end

