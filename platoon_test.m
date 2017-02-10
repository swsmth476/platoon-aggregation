% Use the following model for each platoon:
%
% (For one lead vehicle, two following vehicles)
% (all units in meters, meters/second)
%
%      [v_l ]    [-1 0 0 0 0][v_l ]   [    u_l(t)   ]   [nv_l ]
%   d  [v_f1]    [0 -1 0 0 0][v_f1]   [u_f1(h1,v_f1)]   [nv_f1]
%   -- [v_f2] =  [0 0 -1 0 0][v_f2] + [u_f2(h2,v_f2)] + [nv_f2]   (1)
%   dt [ h1 ]    [1 -1 0 0 0][ h1 ]   [      0      ]   [  0  ]
%      [ h2 ]    [0 1 -1 0 0][ h2 ]   [      0      ]   [  0  ]
%
% Variables:
% ----------
% * v_l = lead vehicle
% * v_f = following vehicle
% * h1 = headway1 (dist. from follower1 -> lead, each vehicle pt. mass)
% * h2 = headway2 (dist. from follower2 -> follower1, each vehicle pt. mass)
% * u_l(t) = input chosen externally (to accomplish some LTL spec, etc. )
% * u_f1(h1, v1) = unidirectional spring pushing back follower 1, with the
%   goal of centering it around a desired time-headway
% * u_f2(h2, v2) = same for follower 2
% * nv_l = lead vehicle nominal velocity
% * nv_f1 = following vehicle 1 nominal velocity
% * nv_f2 = following vehicle 2 nominal velocity

% Platoon 1

% unidirectional virtual springs
time_headway_des1 = 2; %(2s is recommended time headway in normal traffic)
spring1_k = 0.15;
time_headway_des2 = 2;
spring2_k = 0.15;

spring_f1 = spring1_k.*[0 -time_headway_des1 0 1 0];
spring_f2 = spring2_k.*[0 0 -time_headway_des2 0 1];

% nominal velocities
nv_l = 25;
nv_f1 = 25;
nv_f2 = 25; % can modify, with effects to following distances

% go without nominal velocity
A_sub = [-1 0 0 0 0;
    ([0 -1 0 0 0] + spring_f1);
    ([0 0 -1 0 0] + spring_f2);
    1 -1 0 0 0;
    0 1 -1 0 0];

B_sub = [1; 0; 0; 0; 0];

K_sub = [nv_l; nv_f1; nv_f2; 0; 0];

% sanity check
% tspan = [0 60];
% x0 = [25; 27; 28; 27*3; 28*4];
% [t,x] = ode45(@(t,x) pltsys(t,x,A1,B1,K1), tspan, x0);

% Platoon 2 - identical

% Combined system

% Add - additional state for headway between platoons
% Add - additional spring between platoons

A_c = blkdiag(A_sub, 0, A_sub);
B_c = [ [B_sub; zeros(6,1)] [zeros(6,1); B_sub] ];
K_c = [K_sub; 0; K_sub];

th_des_platoon = 2;
spring_k_platoon = 0.15;

spring_platoon = ...
        spring_k_platoon.*[0 0 0 0 0 1 -th_des_platoon 0 0 0 0];
    
A_c(6,:) = spring_platoon;

% Define a transformation to the following 3 states

% va_l = average velocity for lead platoon
% va_f = average (aggregated) velocity for following platoon
% a_h = distance from lead vehicle of following platoon, to follower2 of
%       lead platoon

P_sword = [1/3 1/3 1/3 0 0 0 0 0 0 0 0;
            0 0 0 0 0 0 1/3 1/3 1/3 0 0;
            0 0 0 0 0 1 0 0 0 0 0];
        
P = pinv(P_sword);

% Abstraction system

F = P_sword*A_c*P;
G = P_sword*B_c;
L = P_sword*K_c;
error = P*F - A_c*P;

% discretize w/ dt = 0.1s

dt = 0.1;
F_s = @(s) expm(s*F);
F_d = F_s(dt);
G_d = integral(F_s, 0, dt, 'ArrayValued', true) * G;
L_d = integral(F_s, 0, dt, 'ArrayValued', true) * L;

% define safe set to be 2s < time-headway < 3.5s
time_headway_lo = 2;
time_headway_hi = 3.5;

% keep lead/following vehicle in between 22 and 28 m/s
pl_lo = 22;
pl_hi = 28;
pf_lo = 22;
pf_hi = 28;

Hz = [1 0 0;
    -1 0 0;
    0 1 0;
    0 -1 0;
    0 -time_headway_hi 1;
    0 time_headway_lo -1];
hz = [pl_hi; pl_lo; pf_hi; pf_lo; 0; 0];
Z = Polyhedron(Hz, hz);

% set input bounds to generic braking/acceleration = [-3, 2] m/s^2
% do for each platoon
Hv_sub = [1; -1];
Hv = blkdiag(Hv_sub, Hv_sub);
hv_sub = [2; 3];
hv = [hv_sub; hv_sub];

% find invariant set for two platoons
ls = LinSys(Hv, hv, F_d, G_d, zeros(3,0), L_d);
ls.setd(zeros(3,0), zeros(3,1));
C = ls.ConInvOI(Z);

% invariant set converges - need to double check result