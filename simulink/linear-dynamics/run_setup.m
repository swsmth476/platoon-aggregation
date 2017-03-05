function mdl = run_setup
% mdl = struct of necessary model parameters for simulation

% nominal velocities
mdl.v0L1 = 25;
mdl.v0L2 = 25;
mdl.v01 = 25;
mdl.v02 = 25;
mdl.v03 = 25;
mdl.v04 = 25; % each vehicle has the same nominal velocity
mdl.dh = 50; % desired headway
mdl.k = 5; % gain for spring

% concrete system dynamics %
A_sub = [0 1 0 0 0 0;
        0 -1 0 0 0 0;
        0 0 0 1 0 0;
        mdl.k 0 -mdl.k -1 0 0;
        0 0 0 0 0 1;
        0 0 mdl.k 0 -mdl.k -1]; % dynamics for one platoon subsystem
A = blkdiag(A_sub, A_sub);
A(8,:) = A(8,:) + [0 0 0 0 mdl.k 0 -mdl.k 0 0 0 0 0]; % add spring between platoons
mdl.A = A;
B_sub = [0; 1; 0; 0; 0; 0];
mdl.B = blkdiag(B_sub, B_sub); % control input
mdl.theta = [0; mdl.v0L1; 0; mdl.v01 - mdl.k*mdl.dh; 0; mdl.v02 - mdl.k*mdl.dh; ...
          0; mdl.v0L2 - mdl.k*mdl.dh; 0; mdl.v03 - mdl.k*mdl.dh; 0; mdl.v04 - mdl.k*mdl.dh];
        % constant dynamics

% reference system dynamics %
mdl.F = [0 1 0 0;
    0 -1 0 0;
    0 0 0 1;
    0 0 0 -1]; % autonomous dynamics
mdl.G = [0 0;
        1 0;
        0 0
        0 1]; % control input
mdl.theta_hat = [0; mdl.v0L1; 0; mdl.v0L2]; % constant dynamics

% coordinate transformation %
P_sub = [1 0;
        0 1;
        1 0;
        0 1;
        1 0;
        0 1];
mdl.P = blkdiag(P_sub, P_sub); % linear part
omega_sub = [0; 0; -mdl.dh; 0; -2*mdl.dh; 0];
mdl.omega = [omega_sub; omega_sub]; % constant part

% invariant manifold "error" parameters %
mdl.D = mdl.A*mdl.P - mdl.P*mdl.F;
mdl.E = mdl.A*mdl.omega + mdl.theta - mdl.P*mdl.theta_hat;

% get ISS lyapunov matrices %
% output we will use is velocity of each vehicle and intervehicle headways
% C_sub = [0 1 0 0 0 0;
%         1 0 -1 0 0 0;
%         0 0 0 1 0 0;
%         0 0 1 0 -1 0;
%         0 0 0 0 0 1];
% mdl.C = [C_sub zeros(5,6);
%         0 0 0 0 1 0 -1 0 0 0 0 0;
%         zeros(5,6) C_sub];
M = sdpvar(12,12,'symmetric');
K = sdpvar(2,12);
F = [M >= 0];
F = [F, (mdl.A + mdl.B*K)'*M + M*(mdl.A+mdl.B*K) <= 0];
optimize(F, [], sdpsettings('solver','penlab'));
mdl.M = value(M);
mdl.K = value(K);

end