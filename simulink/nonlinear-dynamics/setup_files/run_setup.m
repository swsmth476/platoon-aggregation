function mdl = run_setup
% mdl = struct of model parameters for simulation

% tank areas %
mdl.A1 = 10;
mdl.A2 = 10;
mdl.A3 = 10;
mdl.A4 = 10;

% pipe constants %
mdl.k1 = 1.5;
mdl.k2 = 1.5;
mdl.k3 = 1.5;

% leaking coefficient %
mdl.gamma = 0.01; % gamma > 0

% full tank initial state %
mdl.x0 = [5; 5; 5; 5];

% reference tank initial state %
mdl.z0 = [5; 5];

% manifold matrix P
mdl.P = [1 0;
        1 0;
        0 1;
        0 1];

% set feedback gain
feedback_gain = 1.5;
mdl.K = -feedback_gain*eye(4);

%%% CLOSED LOOP CONTROLLER %%%
M = 1e4;
mdl.mpc_H = 20;
mdl.mpc_P = -M*ones(mdl.mpc_H,1);
mdl.ut_old = [];
mdl.xt_old = [];
mdl.zt = [];