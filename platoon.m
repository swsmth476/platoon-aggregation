% Dynamics
% d/dt xL = vL
% d/dt vL = -vL + v0
% d/dt x1 = v1
% d/dt v1 = -v1 + v01 + g(xL - x1)
% d/dt x2 = v2
% d/dt v2 -v2 + v02 + g(x1 - x2)

% let g(x) = unidirectional spring = log(x)

v0L1 = 32; % lead vehicle 1 nominal velocity
v0L2 = 
v01 = 25;
v02 = 25;

dh = 50; % desired headway
k = 5; % gain for spring

x0 = [300; 0; 250; 0; 200; 0; 150; 0; 100; 0; 50; 0];
z0 = [300; 0; 150; 0];

[t1,y] = ...
    ode45(@(t1,y) NLPlatoonFull(t1,y,v0,v01,v02,dh,k), ...
    [0, 20], x0);
[t2,w] = ode45(@(t2,w) NLPlatoonAbs(t2,w,v0,desired_headway,gain), ...
            [0, 20], z0);