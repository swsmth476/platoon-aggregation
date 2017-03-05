function par = set_parameters
% set all necessary parameters for platoon simulation

% nominal velocities
par.v0L1 = 25;
par.v0L2 = 25;
par.v01 = 25;
par.v02 = 25;
par.v03 = 25;
par.v04 = 25; % each vehicle has the same nominal velocity
par.dh = 50; % desired headway
par.k = 5; % gain for spring

% concrete system dynamics %
A_sub = [0 1 0 0 0 0;
        0 -1 0 0 0 0;
        0 0 0 1 0 0;
        par.k 0 -par.k -1 0 0;
        0 0 0 0 0 1;
        0 0 par.k 0 -par.k -1]; % dynamics for one platoon subsystem
A = blkdiag(A_sub, A_sub);
A(8,:) = A(8,:) + [0 0 0 0 par.k 0 -par.k 0 0 0 0 0]; % add spring between platoons
par.A = A;
B_sub = [0; 1; 0; 0; 0; 0];
par.B = blkdiag(B_sub, B_sub); % control input
par.theta = [0; par.v0L1; 0; par.v01 - par.k*par.dh; 0; par.v02 - par.k*par.dh; ...
            0; par.v0L2 - par.k*par.dh; 0; par.v03 - par.k*par.dh; 0; par.v04 - par.k*par.dh];
        % constant dynamics

% reference system dynamics %
par.F = [0 1 0 0;
    0 -1 0 0;
    0 0 0 1;
    0 0 0 -1]; % autonomous dynamics
par.G = [0 0;
        1 0;
        0 0
        0 1]; % control input
par.theta_hat = [0; par.v0L1; 0; par.v0L2]; % constant dynamics

% coordinate transformation %
P_sub = [1 0;
        0 1;
        1 0;
        0 1;
        1 0;
        0 1];
par.P = blkdiag(P_sub, P_sub); % linear part
omega_sub = [0; 0; -par.dh; 0; -2*par.dh; 0];
par.omega = [omega_sub; omega_sub]; % constant part

% invariant manifold "error" parameters %
par.D = par.A*par.P - par.P*par.F;
par.E = par.A*par.omega + par.theta - par.P*par.theta_hat;

% get ISS lyapunov matrices %
% output we will use is velocity of each vehicle and intervehicle headways
C_sub = [0 1 0 0 0 0;
        1 0 -1 0 0 0;
        0 0 0 1 0 0;
        0 0 1 0 -1 0;
        0 0 0 0 0 1];
par.C = blkdiag(C_sub);
M_bar = sdpvar(12,12);
K_bar = sdpvar(12,12);
F = [

end