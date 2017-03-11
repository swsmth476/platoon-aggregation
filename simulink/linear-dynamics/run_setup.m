function mdl = run_setup
% mdl = struct of model parameters for simulation

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

mdl.theta = [0; mdl.v0L1; 0; mdl.v01 - mdl.k*mdl.dh; ...
             0; mdl.v02 - mdl.k*mdl.dh; 0; mdl.v0L2 - mdl.k*mdl.dh; ...
             0; mdl.v03 - mdl.k*mdl.dh; 0; mdl.v04 - mdl.k*mdl.dh]; % constant dynamics

C_sub = [0 1 0 0 0 0;
        1 0 -1 0 0 0;
        0 0 0 1 0 0;
        0 0 1 0 -1 0;
        0 0 0 0 0 1];
mdl.C = [C_sub zeros(size(C_sub));
        0 0 0 0 1 0 -1 0 0 0 0 0;
        zeros(size(C_sub)) C_sub]; % outputs are velocities, headways

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

% get discretized reference dynamics for MPC %
dt = 0.1; % timestep
mdl.Fd = expm(mdl.F*dt);
F_s = @(s) expm(s*mdl.F);
mdl.Gd = integral(F_s, 0, dt, 'ArrayValued', true) * mdl.G;
mdl.theta_hatd = integral(F_s, 0, dt, 'ArrayValued', true) * mdl.theta_hat;

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
    
% linear state feedback, Lyapunov %

lambda = .2; % decay rate that we want to achieve

% find Lyapunov matrix M and linear feedback K to achieve this decay
[mdl.M, mdl.K] = decay_rate(mdl.A, mdl.B, mdl.C, lambda);

% sanity check
assert(min(eig(mdl.M - mdl.C'*mdl.C)) >= 0)
assert(min(eig((mdl.A + mdl.B*mdl.K)'*mdl.M ...
    + mdl.M*(mdl.A + mdl.B*mdl.K) + 2*lambda*mdl.M)) <= 0)

% initial states %
mdl.x0 = [300; 25; 250; 25; 200; 25; 150; 25; 100; 25; 50; 25];
mdl.z0 = [300; 25; 150; 25]; % initial conditions

% goal state for reference output %
mdl.H = [0 1 0 0;
        1 0 -1 0;
        0 0 0 1];
mdl.wg = [25; 150; 25];

end