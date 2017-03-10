function [M, K] = decay_rate(A, B, C, rate)
% Find Lyapunov matrix M and state feedback K to achieve a decay rate
% specified by the function input rate, i.e.
% 
% find M, K
% s.t. M >= C'*C
%      (A + B*K)'*M + M*(A + B*K) <= -2*lambda*M

% define new variables M_bar = M^(-1) and K_bar = K*M^(-1)
M_bar = sdpvar(12,12,'symmetric');
K_bar = sdpvar(2,12);

% set up constraints and solve
k = size(C,1);
G = [M_bar M_bar*C';
    C*M_bar eye(k)];
F = [G >= 0];
F = [F, M_bar*A' + A*M_bar + K_bar'*B' + B*K_bar <= -2*rate*M_bar];
optimize(F, [], sdpsettings('solver','mosek'));

M = inv(value(M_bar));
K = value(K_bar)*M;

end