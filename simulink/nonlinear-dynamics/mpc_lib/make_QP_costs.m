function [Q_bar, q_bar, R_bar, r_bar] = make_QP_costs(T,Q,Qf,q,qf,R,r)
%%% Summary %%%
% Returns blocked state/input costs

%%% Description %%%
% Returns  Q_bar = diag(Q, Q, ..., Q, Qf)
%           q_bar = [q; q; ...; q; qf]
%           R_bar = diag(R, R, ..., R)
%           r_bar = [r; r; ...; r]
%

Q_bar = [];
q_bar = [];
R_bar = [];
r_bar = [];

for i = 1:T-1
    Q_bar = blkdiag(Q_bar, Q);
    R_bar = blkdiag(R_bar, R);
    q_bar = [q_bar; q];
    r_bar = [r_bar; r];
end

R_bar = blkdiag(R_bar, R);
r_bar = [r_bar; r];

% add final state costs %
Q_bar = blkdiag(Q_bar, Qf);
q_bar = [q_bar; qf];

end
