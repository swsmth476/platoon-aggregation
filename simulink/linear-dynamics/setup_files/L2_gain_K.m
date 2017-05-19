function K = L2_gain_K(A, B, C, W)
% Find state feedback K to minimize the system L2 gain

gamma = 1.2;
Q = sdpvar(12,12);
Y = sdpvar(2,12); % SDP variables from Boyd LMI book
K = sdpvar(2,12); % extra variable to ensure A+BK is Hurwitz

k = size(C,1);
G = [A*Q + Q*A' + B*Y + Y'*B' + W*W', (C*Q)';
    C*Q, -gamma^2*eye(k)];
F = [G <= 0];
F = [F, A + B*K <= 0];
F = [F, K*Q <= Y];
F = [F, Y <= K*Q];
optimize(F, [], sdpsettings('solver', 'mosek'));

K = value(K);

end