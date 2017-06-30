function [M, K, e_max] = Linf_gain_K(A, B, F, val_max, d_max)
% Consider dynamics of the form
%
% d/dt e = (A + B*K)*e + F*d
%
% Find Lyapunov matrix M and state feedback K such that
%
% e^T P e >= val_max and d^T d >= d_max
%
% implies e^T*(A + B*K)^T*M*e + e^T*M*(A + B*K)*e <= 0        (1)

% define new variables M_bar = M^(-1) and K_bar = K*M^(-1)
M_bar = sdpvar(12,12,'symmetric');
K_bar = sdpvar(2,12);

% make sure M_bar is positive definite
epsilon = 1e-5;
cnstr = [M_bar >= epsilon*eye(size(M_bar))];

% add constraint (1)
k = size(F,2);
alpha = 1;
H = [A*M_bar + M_bar*A' + K_bar'*B' + B*K_bar + alpha*M_bar, F;
    F', -alpha*val_max/d_max*eye(k)];
cnstr = [cnstr, H <= 0];

% add objective (maximize minimum eigenvalue)
t = sdpvar;
cnstr = [cnstr, M_bar <= -t*eye(size(M_bar))];

% bisection(cnstr, t, sdpsettings('solver','mosek'));
bisection(cnstr, t, sdpsettings('solver','mosek'));

% get lyapunov and feedback matrices
M = inv(value(M_bar));
K = value(K_bar)*M;

% find bound
lambda_min = min(eig(M));
e_max = sqrt(val_max/lambda_min);

end