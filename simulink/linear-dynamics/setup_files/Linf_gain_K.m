function [ output_args ] = Linf_gain_K(A, B, C, F, rate)
% blah

% define new variables M_bar = M^(-1) and K_bar = K*M^(-1)
M_bar = sdpvar(12,12,'symmetric');
M = sdpvar(12,12,'symmetric');
K_bar = sdpvar(2,12);

% 
alpha

% set up constraints and solve
k = size(C,1);
G = [M_bar M_bar*C';
    C*M_bar eye(k)];
cnstr = [G >= 0];
cnstr = [cnstr, M_bar*A' + A*M_bar + K_bar'*B' + B*K_bar <= -2*rate*M_bar];
H = [(gain - 2*lambda)*M M*F;
    F'*M -alpha*M];
cnstr = [cnstr, H <= 0];
optimize(cnstr, [], sdpsettings('solver','mosek'));

M = inv(value(M_bar));
K = value(K_bar)*M;

end

