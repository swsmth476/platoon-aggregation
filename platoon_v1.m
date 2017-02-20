% Use the following model for each platoon:
%
% (For one lead vehicle, two following vehicles)
% (all units in meters, meters/second)
%
%      [xld]    [0  1  0  0  0  0][xld]   [      0      ]   [  0  ]
%   d  [vld]    [0  0  0  0  0  0][vld]   [    uld(t)   ]   [  0  ]
%   -- [x1 ] =  [0  0  1  0  0  0][x1 ] + [      0      ] + [  0  ]   (1)
%   dt [v1 ]    [0  0 -1  0  0  0][v1 ]   [  u1(xld,x1) ]   [nv_1 ]
%      [x2 ]    [0  0  0  1  0  0][x2 ]   [      0      ]   [  0  ]
%      [v2 ]    [0  0  0 -1  0  0][v2 ]   [  u2(x1,x2)  ]   [nv_2 ]
%
% Variables:
% ----------
% * xld = lead vehicle position
% * vld = lead vehicle velocity
% * x1 = follower vehicle 1 position
% * v1 = follower vehicle 1 velocity
% * x2 = follower vehicle 2 position
% * v2 = follower vehicle 2 velocity
% * uld(t) = input chosen externally (to accomplish some LTL spec, etc. )
% * u1(xld,x1) = unidirectional spring pushing back follower 1, with the
%   goal of centering it around a desired headway
% * u2(x1,x2) = same for follower 2
% * nv_l = lead vehicle nominal velocity
% * nv_f1 = follower vehicle 1 nominal velocity
% * nv_f2 = follower vehicle 2 nominal velocity

% Platoon 1

% unidirectional virtual springs
d1 = 50; % desired following distance
d2 = 50;
k1 = 1; % "spring constant" or gain
k2 = 1;

spring_f1 = k1.*[1 0 -1 0 0 0];
spring_f2 = k2.*[0 0 1 0 -1 0];

% nominal velocities
nv_l = 25;
nv_f1 = 25;
nv_f2 = 25; % can modify, with effects to following distances

% go without nominal velocity
A_sub = [0 1 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 1 0 0;
        k1 0 -k1 -1 0 0;
        0 0 0 0 0 1;
        0 0 k2 0 -k2 -1];

B_sub = [0; 1; 0; 0; 0; 0];

K_sub = [0; nv_l; 0; nv_f1 - k1*d1; 0; nv_f2 - k2*d2];

% Platoon 2 - identical

% Combined system
% Add - additional spring between platoons

A_c = blkdiag(A_sub, A_sub);
B_C = blkdiag(B_sun, B_sub);
K_c = [K_sub; K_sub];

d_platoon = 50;
k_platoon = 1;

spring_platoon = ...
        k_platoon.*[0 0 0 0 1 0 -1 0 0 0 0 0];
    
A_c(8,:) = A_c(8,:) + spring_platoon;
K_c(8,:) = K_c(8,:) - d_platoon*k_platoon;

% Define a transformation to the following 4 states:

% x_l1 = lead platoon position
% v_l1 = lead platoon velocity
% x_l2 = following platoon position
% v_l2 = following platoon velocity

% Abstraction system

F = P_sword*A_c*P;
G = P_sword*B_c;
L = P_sword*K_c;
error = P*F - A_c*P;

% use Q to cancel out error as much as possible
Q = B_c\error;

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
pl_lo = 20;
pl_hi = 30;
pf_lo = 20;
pf_hi = 30;

% speed difference bound
v_delta_max = 2.5;

Hz = [1 0 0;
    -1 0 0;
    0 1 0;
    0 -1 0;
    0 -time_headway_hi 1;
    0 time_headway_lo -1;
    1 -1 0;
    -1 1 0];
hz = [pl_hi; -pl_lo; pf_hi; -pf_lo; 0; 0; v_delta_max; v_delta_max];
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

% save model parameters for Simulink %

% 3-dimensional abstraction - continuous
model.F = F;
model.G = G;
model.L = L;

% 3-dimensional abstraction - discrete
model.F_d = F_d;
model.G_d = G_d;
model.L_d = L_d;

% 11-dimensional model - continuous
model.A = A_c;
model.B = B_c;
model.K = K_c;

% 