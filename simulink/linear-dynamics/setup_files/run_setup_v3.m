function mdl = run_setup_v3
% mdl = struct of model parameters for simulation

%%% model parameters %%%
mdl.v_0 = 28;
mdl.dh = 12; % desired headway
% mdl.tau = 10 / 28; % desired time headway
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
     mdl.v_0 - va + ua;
     vb; 
     mdl.v_0 - vb + ub];
Ac = double(jacobian(f, [xa va xb vb]));
Bc = double(jacobian(f, [ua ub]));
Kc = [0; mdl.v_0; 0; mdl.v_0]; % constant dynamics
mdl.Ad = expm(Ac*dt);
A_s = expm(Ac*s);
mdl.Bd = double(int(A_s, s, [0 dt])*Bc);
mdl.Kd = double(int(A_s, s, [0 dt])*Kc);

%%% manifold map %%%
P_sub = [eye(2); eye(2)];
mdl.P = blkdiag(P_sub, P_sub); % linear part
omega_sub = [0; 0; -mdl.dh; 0];
mdl.omega = [omega_sub; omega_sub]; % constant part

%%% initial states %%%
% initial distance between platoons is 22 => decrease to 12
mdl.x0 = [58; 28; 46; 28; 24; 28; 12; 28];
mdl.z0 = [58; 28; 24; 28];
% initial distance between platoons is 27 => decrease to 12
% initial velocity of the platoon is 25 => increase to 30
% mdl.x0 = [135; 25; 123; 25; 111; 25; 99; 25; 87; 25; ...
%           60; 25; 48; 25; 36; 25; 24; 25; 12; 25];
% mdl.z0 = [135; 25; 60; 25];
% mdl.x0 = [40; 28; 30; 28; 20; 28; 10; 28];
% mdl.z0 = [40; 28; 20; 28];

% for simulink initialization
mdl.init = 0;

%%% CLOSED LOOP CONTROLLER %%%
M = 1e4;
% change mpc_H to 20 for 1st example, to 30 for 2nd example
mdl.mpc_H = 30;
mdl.mpc_P = -M*ones(mdl.mpc_H,1);
mdl.ut_old = [];
mdl.zt = [];
mdl.init_step = false; % to fix weird simulation initialization bug

end