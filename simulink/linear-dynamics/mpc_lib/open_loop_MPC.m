function [ output_args ] = open_loop_MPC(A,B,theta,x0,T,Q,Qf,q,qf,R,r,Hu,hu,Hx,hx,P,Ut_old)
%%% Summary %%%

%%% Description %%%

constraints = [];

%%% SYSTEM_CONSTRAINTS %%%
n = size(x0,1); % state dimension
m = size(B,2); % input dimension
x_bar = []; % x = [x(t+1); x(t+2); ... x(t+T)]
u_bar = []; % u = [u(t); u(t+1); ... u(t+T-1)] (both col vectors)

% create state decision variables for each time index
for i = 1:T
    x{i} = sdpvar(n,1);
    u{i} = sdpvar(m,1);
    x_bar = [x_bar; x{i}];
    u_bar = [u_bar, u{i}];
end

% require that system updates satisfy x(t+1) = Ax(t) + Bu(t) + theta
[G, M] = make_sys_constr(T, A, B, theta, x0);
constraints = [constraints, x_bar <= G*u_bar + M];
constraints = [constraints, x_bar >= G*u_bar + M];

%%% INPUT_CONSTRAINTS %%%
% require that inputs satisfy Hu*u(t) <= hu for all t
[Hu_bar, hu_bar] = make_input_constr(T, Hu, hu);
constraints = [constraints, Hu_bar*u_bar <= hu_bar];

%%% STL_CONSTRAINTS %%%
% (a quantitative STL encoding is used - allows robustness of formula satisfaction)
%
% STL formula predicates %
% mu1: platoon headway lower bound
% mu2: platoon headway upper bound
% mu3: vehicle 1 target speed lower bound
% mu4: vehicle 1 target speed upper bound
% mu5: vehicle 2 target speed lower bound
% mu6: vehicle 2 target speed upper bound

num_pred = 6;

% STL formula values %
headway_lb = 140;
headway_ub = 160;
vel_lb = 29;
vel_ub = 31;

% introduce predicate variables 
for i = 1:num_pred
    rt_mu{i} = sdpvar(T,1);
end

% predicates are linear, of the form mu(x_t) = a*x_t + b %
mu_a = zeros(num_pred, n);
mu_b = zeros(num_pred, 1);

mu_a(1,:) = [1 0 -1 0 0 0];
mu_a(2,:) = [-1 0 1 0 0 0];
mu_a(3,:) = [0 1 0 0 0 0];
mu_a(4,:) = [0 -1 0 0 0 0];
mu_a(5,:) = [0 0 0 1 0 0];
mu_a(6,:) = [0 0 0 -1 0 0];

mu_b(1) = -headway_lb;
mu_b(2) = headway_ub;
mu_b(3) = -vel_lb;
mu_b(4) = vel_ub;
mu_b(5) = -vel_lb;
mu_b(6) = vel_ub;

% set rt_mu(i) = mu_i(x_t) for each predicate, and time index
for i = 1:num_pred
    for j = 0:T-1
        x_idx = (j*n+1):(j*n+n);
        constraints = [constraints, rt_mu{i}(j+1) <= mu_a(i,:)*x(x_idx) + mu_b(i)];
        constraints = [constraints, rt_mu{i}(j+1) >= mu_a(i,:)*x(x_idx) + mu_b(i)];
    end
end

% define two variables as the conjunction of the predicates %
% phi_t = mu_1(x_t) ^ mu_2(x_t)
% psi_t = mu_3(x_t) ^ mu_4(x_t) ^ mu_5(x_t) ^ mu_6(x_t)
rt_phi = sdpvar(T,1);
rt_psi = sdpvar(T,1);

num_phi = 2;
num_psi = 4; % number of conjunctions for each variable

for i = 1:T
    % create binary variables for conjunctions
    pt_phi{i} = binvar(2,1);
    pt_psi{i} = binvar(4,1);
    % add conjunction constraints
    constraints = [constraints, sum(pt_phi{i}) <= 1];
    constraints = [constraints, sum(pt_phi{i}) >= 1];
    constraints = [constraints, sum(pt_psi{i}) <= 1];
    constraints = [constraints, sum(pt_psi{i}) >= 1];
    for j = 1:num_phi
    constraints = [constraints, rt_phi(i) <= rt_mu{j}(i)];
    constraints = [constraints, rt_mu{j}(i) - (1 - pt_phi{i}(j))*M <= rt_phi(i)];
    constraints = [constraints, rt_phi(i) <= 
    end
    for j = 1:num_psi
    constraints = [constraints, rt_psi(i) <= rt_mu{2+j}(i)];
    constraints = [constraints, 
    end
end

end