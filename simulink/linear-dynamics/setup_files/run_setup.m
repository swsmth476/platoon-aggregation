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

mdl.H = [0 1 0 0;
        1 0 -1 0;
        0 0 0 1];

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
mdl.d = mdl.D(8,:);
mdl.E = mdl.A*mdl.omega + mdl.theta - mdl.P*mdl.theta_hat;
mdl.e = mdl.E(8);

% "disturbance" matrix %
temp = eye(12);
e8 = temp(:,8);
mdl.R = eye(2);
mdl.W = [e8 mdl.B*mdl.R - mdl.P*mdl.G];
% find bound on disturbance %
headway_delta = 5; % (meters)
input_max = 3.5; % (m/s^2)
d_max = norm([headway_delta input_max input_max]')^2; % one for each vehicle

% linear state feedback, Lyapunov %

% find Lyapunov matrix M and linear feedback K to achieve this decay
% and minimize closed loop L-infinity gain (see Linf_gain_K)
[mdl.M, mdl.K, mdl.e_max] = ...
    Linf_gain_K(mdl.A, mdl.B, mdl.W, 1500, d_max);

% sanity check
% assert(min(eig(mdl.M - mdl.C'*mdl.C)) >= 0)
% assert(min(eig((mdl.A + mdl.B*mdl.K)'*mdl.M ...
%    + mdl.M*(mdl.A + mdl.B*mdl.K) + 2*rate*mdl.M)) <= 0)

% initial states %
mdl.x0 = [300; 25; 250; 25; 200; 25; 150; 25; 100; 25; 50; 25];
mdl.z0 = [300; 25; 150; 25];

% goal state for reference output %
mdl.H = [0 1 0 0;
        1 0 -1 0;
        0 0 0 1];
mdl.wg = [30; 150; 30];

% for simulink initialization
mdl.init = 0;

%%% CLOSED LOOP CONTROLLER %%%
M = 1e4;
% change mpc_H to 20 for 1st example, to 30 for 2nd example
mdl.mpc_H = 35;
mdl.mpc_P = -M*ones(mdl.mpc_H,1);
mdl.ut_old = [];
mdl.zt = [];

%%% OPEN LOOP CONTROLLER %%%

% create augmented system dynamics
A = [[mdl.Fd mdl.Gd]; zeros(2,4) eye(2)];
B = [zeros(4,2); eye(2)];
theta = [mdl.theta_hatd; zeros(2,1)];

% augmented initial state
z0 = [mdl.z0; zeros(2,1)];

% choose augmented system state/input costs
Q = zeros(6);
Qf = zeros(6);
q = zeros(6,1);
qf = zeros(6,1);
R = eye(2);
r = zeros(2,1);

% jerk constraints
j_ub = 0.3;
j_lb = -0.3;

% input constraints
Hu = [eye(2); -eye(2)];
hu = [j_ub; j_ub; -j_lb; -j_lb];

% run open-loop MPC test
% T = 75;
% mdl.u_opt = open_loop_MPC(A, B, theta, z0, T, ...
%                        Q, Qf, q, qf, R, r, Hu, hu);

end