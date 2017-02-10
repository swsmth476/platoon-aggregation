%% Use the following model for each platoon:
%
% For one lead vehicle, two following vehicles:
%
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


%% Platoon 1

% unidirectional virtual springs
time_headway_des1 = 2; %(2s is recommended time headway in normal traffic)
spring1_gain = 0.15;
time_headway_des2 = 2;
spring2_gain = 0.15;

spring_f1 = spring1_gain.*[0 -time_headway_des1 0 1 0];
spring_f2 = spring2_gain.*[0 0 -time_headway_des2 0 1];

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

%% Combined system

% Add - additional state for headway between platoons
% Add - additional spring between platoons

A_c = blkdiag(A_sub, 0, A_sub);
B_c = [ [B_sub; zeros(6,1)] [zeros(6,1); B_sub] ];
K_c = [K_sub; 0; K_sub];

th_des_platoon = 2;
spring_gain_platoon = 0.15;

%% Define a transformation to the following 3 states

% va_l = average velocity for lead platoon
% va_f = average (aggregated) velocity for following platoon
% a_h = distance from lead vehicle of following platoon, to follower2 of
%       lead platoon

%P_sword = 
        
% P = pinv(P_sword);
            
