% Use the following model for each platoon:
%
% (For one lead vehicle, two following vehicles)
% (all units in meters, meters/second)
%
%      [xL]    [0  1  0  0  0  0][xld]   [      0      ]   [  0  ]
%   d  [vL]    [0  0  0  0  0  0][vld]   [    uL(t)   ]   [  0  ]
%   -- [x1] =  [0  0  1  0  0  0][x1 ] + [      0      ] + [  0  ]   (1)
%   dt [v1]    [0  0 -1  0  0  0][v1 ]   [  u1(xld,x1) ]   [ v01 ]
%      [x2]    [0  0  0  1  0  0][x2 ]   [      0      ]   [  0  ]
%      [v2]    [0  0  0 -1  0  0][v2 ]   [  u2(x1,x2)  ]   [ v02 ]
%
% Variables:
% ----------
% * xL = lead vehicle position
% * vL = lead vehicle velocity
% * x1 = follower vehicle 1 position
% * v1 = follower vehicle 1 velocity
% * x2 = follower vehicle 2 position
% * v2 = follower vehicle 2 velocity
% * uL(t) = input chosen externally (to accomplish some LTL spec, etc. )
% * u1(xld,x1) = unidirectional spring pushing back follower 1, with the
%   goal of centering it around a desired headway
% * u2(x1,x2) = same for follower 2
% * v01 = follower vehicle 1 nominal velocity
% * v02 = follower vehicle 2 nominal velocity

% Platoon 1

% unidirectional virtual springs
% using, e.g., u2(x1,x2) = k(x1 - x2 - d2)

d1 = 50; % desired following distance
d2 = 50;
k1 = 1; % "spring constant" or gain
k2 = 1;

% nominal velocities
v01 = 25;
v02 = 25; % can modify, with effects to following distances

A_sub = [0 1 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 1 0 0;
        k1 0 -k1 -1 0 0;
        0 0 0 0 0 1;
        0 0 k2 0 -k2 -1];

B_sub = [0; 1; 0; 0; 0; 0];

K_sub = [0; 0; 0; v01 - k1*d1; 0; v02 - k2*d2];

% Platoon 2 - identical

% Combined system
% Add - additional spring between platoons

A = blkdiag(A_sub, A_sub);
B = blkdiag(B_sub, B_sub);
K = [K_sub; K_sub];

d_platoon = 50;
k_platoon = 1;

spring_platoon = ...
        k_platoon.*[0 0 0 0 1 0 -1 0 0 0 0 0];
    
A(8,:) = A(8,:) + spring_platoon;
K(8,:) = K(8,:) - k_platoon*d_platoon;

% Define a transformation for following 4 states:

% x = Pz + omega

% Where z(1 - 4) are the following:
% z1 = xL1 = lead platoon position
% z2 = vL1 = lead platoon velocity
% z3 = xL2 = following platoon position
% z4 = vL2 = following platoon velocity

P_sub = [1 0;
    0 1;
    1, -1/k1;
    0 1;
    1, -1/k1-1/k2;
    0 1];

P = blkdiag(P_sub, P_sub);

omega_sub = [0; 0; -d1 + v01/k1; 0; -d1 - d2 + v01/k1 + v02/k2; 0];

omega = [omega_sub;
         omega_sub];

% Abstraction system

F_sub = [0 1;
         0 0];
     
F = blkdiag(F_sub, F_sub);
     
G_sub = [0;
         1];
     
G = blkdiag(G_sub, G_sub);
    
L_sub = [0;
         0];
     
L = [L_sub;
     L_sub];
     
error = P*F - A*P;

% use Q to cancel out error as much as possible
Q = B\error;

% discretize w/ dt = 0.1s

dt = 0.1;
F_s = @(s) expm(s*F);
F_d = F_s(dt);
G_d = integral(F_s, 0, dt, 'ArrayValued', true) * G;
L_d = integral(F_s, 0, dt, 'ArrayValued', true) * L;

% define safe set to be 6s < time-headway < 8s
th_lo = 6;
th_hi = 8;

% keep lead/following vehicle in between 20 and 30 m/s
pl_lo = 20;
pl_hi = 30;
pf_lo = 20;
pf_hi = 30;

% speed difference bound
v_delta = 2.5;

Hz = [0 1 0 0;
      0 0 0 1;
      0 -1 0 0;
      0 0 0 -1;
      1 0 -1 -th_hi;
      -1 0 1 th_lo;
      0 1 0 -1;
      0 -1 0 1];
hz = [pl_hi; pf_hi; -pl_lo; -pf_lo; 0; 0; v_delta; v_delta];

Z = Polyhedron(Hz, hz);

% set input bounds to generic braking/acceleration = [-3, 2] m/s^2
% do for each platoon
Hv_sub = [1; -1];
Hv = blkdiag(Hv_sub, Hv_sub);
hv_sub = [2; 3];
hv = [hv_sub; hv_sub];

% find invariant set for two platoons
ls = LinSys(Hv, hv, F_d, G_d, zeros(4,0), L_d);
ls.setd(zeros(4,0), zeros(4,1));
C = ls.ConInvOI(Z);

% save model parameters for Simulink %

% % 3-dimensional abstraction - continuous
% model.F = F;
% model.G = G;
% model.L = L;
% 
% % 3-dimensional abstraction - discrete
% model.F_d = F_d;
% model.G_d = G_d;
% model.L_d = L_d;
% 
% % 11-dimensional model - continuous
% model.A = A_c;
% model.B = B_c;
% model.K = K_c;
