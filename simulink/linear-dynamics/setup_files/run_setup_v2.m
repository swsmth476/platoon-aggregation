function mdl = run_setup_v2
% mdl = struct of model parameters for simulation

%%% model parameters %%%
mdl.v_0 = 28;
mdl.dh = 50; % desired headway
mdl.k = 5; % gain for spring
mdl.rho = 0.3245; % drag coefficient
mdl.m = 1722; % vehicle mass

%%% get discretized reference dynamics for MPC %%%
syms xa va xb vb ua ub;
syms s;
% let dynamics be 
% \dot{x} = f(x,u) + theta
% for analysis
dt = 0.1;
f = [va
     mdl.v_0 - va - mdl.rho*va^2/mdl.m + ua;
     vb; 
     mdl.v_0 - vb - mdl.rho*vb^2/mdl.m + ub];
Ac = jacobian(f, [xa va xb vb]);
Bc = double(jacobian(f, [ua ub]));
Kc = [0; mdl.v_0; 0; mdl.v_0]; % constant dynamics
mdl.Ad = expm(Ac*dt);
A_s = expm(Ac*s);
mdl.Bd = int(A_s, s, [0 dt])*Bc;
mdl.Kd = int(A_s, s, [0 dt])*Kc;

%%% manifold map %%%
P_sub = [1 0;
        0 1;
        1 0;
        0 1];
mdl.P = blkdiag(P_sub, P_sub); % linear part
omega_sub = [0; 0; -mdl.dh; 0];
mdl.omega = [omega_sub; omega_sub]; % constant part

%%% initial states %%%
mdl.x0 = [300; 28; 250; 28; 200; 28; 150; 28; 100; 28; 50; 28];
mdl.z0 = [300; 28; 150; 28];

%%% goal state for reference output %%%
mdl.H = [0 1 0 0;
        1 0 -1 0;
        0 0 0 1];
mdl.wg = [30; 150; 30];

% for simulink initialization
mdl.init = 0;

%%% CLOSED LOOP CONTROLLER %%%
M = 1e4;
% change mpc_H to 20 for 1st example, to 30 for 2nd example
mdl.mpc_H = 30;
mdl.mpc_P = -M*ones(mdl.mpc_H,1);
mdl.ut_old = [];
mdl.zt = [];

end