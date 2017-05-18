function u_opt = open_loop_MPC(A,B,theta,x0,T,Q,Qf,q,qf,R,r,Hu,hu)
%%% Summary %%%

%%% Description %%%

% make M a large number
M = 1e4;

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
    u_bar = [u_bar; u{i}];
end

% require that system updates satisfy x(t+1) = Ax(t) + Bu(t) + theta
[G, L] = make_sys_constr(T, A, B, theta, x0);
constraints = [constraints, x_bar <= G*u_bar + L];
constraints = [constraints, x_bar >= G*u_bar + L];

%%% INPUT_CONSTRAINTS %%%
% require that inputs satisfy Hu*u(t) <= hu for all t
[Hu_bar, hu_bar] = make_input_constr(T, Hu, hu);
constraints = [constraints, Hu_bar*u_bar <= hu_bar];

%%% STL_CONSTRAINTS %%%
% (a quantitative STL encoding is used -
%                  allows us to adjust robustness of formula satisfaction)
%
% STL formula predicates %
% mu1: platoon headway lower bound
% mu2: platoon headway upper bound
% mu3: vehicle 1 accel upper bound
% mu4: vehicle 1 accel lower bound
% mu5: vehicle 2 accel upper bound
% mu6: vehicle 2 accel lower bound
% mu7: vehicle 1 target speed lower bound
% mu8: vehicle 1 target speed upper bound
% mu9: vehicle 2 target speed lower bound
% mu10: vehicle 2 target speed upper bound

num_pred = 10;

% STL formula values %
headway_des = 150;
headway_delta = 5;
headway_lb = headway_des - headway_delta;
headway_ub = headway_des + headway_delta;
vel_des = 30;
vel_delta = 1;
vel_lb = vel_des - vel_delta;
vel_ub = vel_des + vel_delta;
accel_bd = 8; % absolute value |accel| < accel_bd

% introduce predicate variables
% predicates are affine, of the form mu(x(i)) = a*x(i) + b %
% rt_mu{i}(j) = predicate j at time index i = mu_j(x(i)) = a_j*x(i) + b_j
for i = 1:T
    rt_mu{i} = sdpvar(num_pred,1);
end

% row 'j' represents 'a_j' from predicate mu_i(x(i))= a_j*x(i) + b_j
mu_a = [1 0 -1 0 0 0;
        -1 0 1 0 0 0;
        0 0 0 0 1 0;
        0 0 0 0 -1 0;
        0 0 0 0 0 1;
        0 0 0 0 0 -1;
        0 1 0 0 0 0;
        0 -1 0 0 0 0;
        0 0 0 1 0 0;
        0 0 0 -1 0 0];

% row 'j' represents 'b_j' from predicate mu_i(x_t) = a_j*x(i) + b_j
mu_b = [-headway_lb;
        headway_ub;
        accel_bd;
        accel_bd;
        accel_bd;
        accel_bd;
        -vel_lb;
        vel_ub;
        -vel_lb;
        vel_ub];

% set rt_mu(i) = mu_i(x_t) for each predicate, and time index
for i = 1:T
    for j = 1:num_pred
        constraints = [constraints, rt_mu{i}(j) <= mu_a(j,:)*x{i} + mu_b(j)];
        constraints = [constraints, rt_mu{i}(j) >= mu_a(j,:)*x{i} + mu_b(j)];
    end
end

% introduce two variables as the conjunctions of the predicates %
% phi_t = mu_1(x_t) ^ mu_2(x_t) ^ ... ^ mu_6(x_t)
% psi_t = mu_7(x_t) ^ mu_8(x_t) ^ mu_9(x_t) ^ mu_10(x_t)

% set acceleration requirement
accel_time = 55;

% variables for phi/psi formulas
rt_phi = sdpvar(T,1);
rt_psi = sdpvar(accel_time,1);

% number of conjunctions for each variable
num_phi = 6;
num_psi = 4;

% NOTE: all the following constraints are simple negations, conjunctions,
% and disjunctions of predicates, based on equations (3), (4), and (5) in 
% "Model Predictive Control for Signal Temporal Logic Specifications"
% by Vasumathi Raman et al.

% do conjunctions necessary for phi
for i = 1:T
    % create binary variables for conjunctions
    pt_phi{i} = binvar(num_phi,1);
    % add conjunction constraints
    constraints = [constraints, sum(pt_phi{i}) <= 1];
    constraints = [constraints, sum(pt_phi{i}) >= 1];
    for j = 1:num_phi
        constraints = [constraints, rt_phi(i) <= rt_mu{i}(j)];
        constraints = [constraints, rt_mu{i}(j) - (1 - pt_phi{i}(j))*M <= rt_phi(i)];
        constraints = [constraints, rt_phi(i) <= rt_mu{i}(j) + M*(1 - pt_phi{i}(j))];
    end
end

% do conjunctions necessary for psi
for i = 1:accel_time
    % create binary variables for conjunctions
    pt_psi{i} = binvar(num_psi,1);
    % add conjunction constraints
    constraints = [constraints, sum(pt_psi{i}) <= 1];
    constraints = [constraints, sum(pt_psi{i}) >= 1];
    for j = 1:num_psi
        constraints = [constraints, rt_psi(i) <= rt_mu{i}(j+num_phi)];
        constraints = [constraints, rt_mu{i}(j+num_phi) - (1 - pt_psi{i}(j))*M <= rt_psi(i)];
        constraints = [constraints, rt_psi(i) <= rt_mu{i}(j+num_phi) + M*(1 - pt_psi{i}(j))];
    end
end

% introduce variable rt_phi_alw = always_[0,T] phi
rt_phi_alw = sdpvar;
pt_phi_alw = binvar(T,1);
constraints = [constraints, sum(pt_phi_alw) <= 1];
constraints = [constraints, sum(pt_phi_alw) >= 1];
for i = 1:T
    constraints = [constraints, rt_phi_alw <= rt_phi(i)];
    constraints = [constraints, rt_phi(i) - (1 - pt_phi_alw(i))*M <= rt_phi_alw];
    constraints = [constraints, rt_phi_alw <= rt_phi(i) + M*(1 - pt_phi_alw(i))];
end

% introduce variable rt_psi_even = eventually_[0,accel_time] psi
rt_psi_even = sdpvar;
pt_psi_even = binvar(accel_time,1);
constraints = [constraints, sum(pt_psi_even) <= 1];
constraints = [constraints, sum(pt_psi_even) >= 1];
for i = 1:accel_time
    constraints = [constraints, rt_psi_even >= rt_psi(i)];
    constraints = [constraints, rt_psi(i) - (1 - pt_psi_even(i))*M <= rt_psi_even];
    constraints = [constraints, rt_psi_even <= rt_psi(i) + M*(1 - pt_psi_even(i))];
end

% introduce variable rt_mpc = rt_phi_alw ^ rt_psi_even
rt_mpc = sdpvar;
pt_mpc = binvar(2,1);
constraints = [constraints, sum(pt_mpc) <= 1];
constraints = [constraints, sum(pt_mpc) >= 1];
constraints = [constraints, rt_mpc <= rt_phi_alw];
constraints = [constraints, rt_mpc <= rt_psi_even];
constraints = [constraints, rt_phi_alw - (1 - pt_mpc(1))*M <= rt_mpc];
constraints = [constraints, rt_mpc <= rt_phi_alw + M*(1 - pt_mpc(1))];
constraints = [constraints, rt_psi_even - (1 - pt_mpc(2))*M <= rt_mpc];
constraints = [constraints, rt_mpc <= rt_psi_even + M*(1 - pt_mpc(2))];

% robustness_margin should be adjusted based on maximum error
% between concrete and reference systems
% (this will depend on the L-infinity gain of the feedback K)
robustness_margin = 0;

% final constraint
constraints = [constraints, rt_mpc >= robustness_margin];

%%% OBJECTIVE_FUNCTION %%%
[Q_bar, q_bar, R_bar, r_bar] = make_QP_costs(T,Q,Qf,q,qf,R,r);
obj_fun = 1/2*(x_bar'*Q_bar*x_bar + u_bar'*R_bar*u_bar) + ...
                q_bar'*x_bar + r_bar'*u_bar;

%%% CALL SOLVER %%%
optimize(constraints, [], sdpsettings('solver','gurobi'))
u_opt = value(u_bar);

end