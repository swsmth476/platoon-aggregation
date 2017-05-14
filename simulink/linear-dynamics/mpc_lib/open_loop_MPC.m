function [ output_args ] = open_loop_MPC(A,B,theta,x0,T,Q,Qf,q,qf,R,r)
%%% Summary %%%
% Adding a description now

%%% Description %%%

constraints = [];

%%% SYSTEM_CONSTRAINTS %%%
n = size(x0,1); % state dimension
m = size(B,2); % input dimension
u = sdpvar(m*T,1); % u = [u(t); u(t+1); ... u(t+T-1)] (both col vectors)
x = sdpvar(n*T,1); % x = [x(t+1); x(t+2); ... x(t+T)] 
[G, M] = make_sys_constr(T, A, B, theta, x0);
constraints = [constraints, x <= G*x + M];
constraints = [constraints, x >= G*x + M];

%%% INPUT_CONSTRAINTS %%%


end