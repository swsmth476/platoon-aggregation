function [u_opt, xt] = open_loop_star(A,B,theta,x0,H,Q,Qf,q,qf,R,r,Hu,hu,P,ut_old,xt_old,signal)
%%% Summary %%%
% Open loop controller to be called by closed loop MPC

%%% Description %%%
% STL formula encoded is:
%
% phi_mpc = phi ^ (signal ==> eventually_[0,H] psi)
%
% where phi encodes safety and input constraints
% and psi encodes velocity constraints

% make M a large number
M = 1e4;

% planning horizon
T = 2*H;

constraints = [];

%%% SYSTEM_CONSTRAINTS %%%
n = size(x0,1); % state dimension
m = size(B,2); % input dimension
x_bar = []; % x = [x(t+1); x(t+2); ... x(t+T)]
u_bar = []; % u = [u(t); u(t+1); ... u(t+T-1)] (both col vectors)

% create state decision variables for each time index

% time horizon is split into previous time steps and future predictions
past_L = size(xt_old, 2);

% variables for each section of the time horizon
x_past = [];
u_past = [];
x_predict = [];
u_predict = [];

% these are time steps that already happened
for i = 1:past_L
    x{i} = sdpvar(n,1);
    u{i} = sdpvar(m,1);
    x_past = [x_past; x{i}];
    u_past = [u_past; u{i}];
end

for i = past_L+1:T
    x{i} = sdpvar(n,1);
    u{i} = sdpvar(m,1);
    x_predict = [x_predict; x{i}];
    u_predict = [u_predict; u{i}];
end

% concatenate the two sections of the time horizon
x_bar = [x_past; x_predict];
u_bar = [u_past; u_predict];

% require that past inputs are respected
for i = 1:size(ut_old,2)
    constraints = [constraints, u{i} <= ut_old(:,i)];
    constraints = [constraints, u{i} >= ut_old(:,i)];
end

% require that past states are respected
for i = 1:size(xt_old,2)
    constraints = [constraints, x{i} <= xt_old(:,i)];
    constraints = [constraints, x{i} >= xt_old(:,i)];
end

%%% UPDATE_CONSTRAINTS %%%
% require that system updates satisfy x(t+1) = Ax(t) + Bu(t) + theta
[G, L] = make_sys_constr(T-past_L, A, B, theta, xt_old(:,past_L));
constraints = [constraints, x_predict <= G*u_predict + L];
constraints = [constraints, x_predict >= G*u_predict + L];

%%% INPUT_CONSTRAINTS %%%
% require that inputs satisfy Hu*u(t) <= hu for all t
[Hu_bar, hu_bar] = make_input_constr(T, Hu, hu);
constraints = [constraints, Hu_bar*u_bar <= hu_bar];

%%% STL_CONSTRAINTS %%%
% (a quantitative STL encoding is used -
%                  allows us to adjust robustness of formula satisfaction)

% get predicates for intended formula
[num_pred, mu_a, mu_b, num_phi, num_psi] = make_predicates;

% rt_mu{i}(j) = predicate j at time index i = mu_j(x(i)) = a_j*x(i) + b_j
for i = 1:T
    rt_mu{i} = sdpvar(num_pred,1);
end

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

% variables for phi/psi formulas
rt_phi = sdpvar(T,1);
rt_psi = sdpvar(T,1);

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
for i = 1:T
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

% introduce variable rt_psi_even = eventually_[0,H] psi
% introduce variable rt_impl = signal ==> rt_psi_even (logical implies)
% introduce variable rt_mpc = rt_phi_alw ^ rt_impl
rt_psi_even = sdpvar(H,1);
rt_impl = sdpvar(H,1);
rt_mpc = sdpvar(H,1);

% invert signal input for implication formula
n_sig = -signal;

for i = 1:H
    
    % eventually
    pt_psi_even{i} = binvar(H,1);
    constraints = [constraints, sum(pt_psi_even{i}) <= 1];
    constraints = [constraints, sum(pt_psi_even{i}) >= 1];
    for j = 1:H
        constraints = [constraints, rt_psi_even(i) >= rt_psi(i+j-1)];
        constraints = [constraints, rt_psi(i+j-1) - (1 - pt_psi_even{i}(j))*M <= rt_psi_even(i)];
        constraints = [constraints, rt_psi_even(i) <= rt_psi(i+j-1) + M*(1 - pt_psi_even{i}(j))];
    end

    % implication
    pt_impl{i} = binvar(2,1);
    constraints = [constraints, sum(pt_impl{i}) <= 1];
    constraints = [constraints, sum(pt_impl{i}) >= 1];
    constraints = [constraints, rt_impl(i) >= rt_psi_even(i)];
    constraints = [constraints, rt_impl(i) >= n_sig(i)];
    constraints = [constraints, rt_psi_even(i) - (1 - pt_impl{i}(1))*M <= rt_impl(i)];
    constraints = [constraints, rt_impl(i) <= rt_psi_even(i) + M*(1 - pt_impl{i}(1))];
    constraints = [constraints, n_sig(i) - (1 - pt_impl{i}(2))*M <= rt_impl(i)];
    constraints = [constraints, rt_impl(i) <= n_sig(i) + M*(1 - pt_impl{i}(2))];
    
    % overall formula
    pt_mpc{i} = binvar(2,1);
    constraints = [constraints, sum(pt_mpc{i}) <= 1];
    constraints = [constraints, sum(pt_mpc{i}) >= 1];
    constraints = [constraints, rt_mpc(i) <= rt_phi(i)];
    constraints = [constraints, rt_mpc(i) <= rt_impl(i)];
    constraints = [constraints, rt_phi(i) - (1 - pt_mpc{i}(1))*M <= rt_mpc(i)];
    constraints = [constraints, rt_mpc(i) <= rt_phi(i) + M*(1 - pt_mpc{i}(1))];
    constraints = [constraints, rt_impl(i) - (1 - pt_mpc{i}(2))*M <= rt_mpc(i)];
    constraints = [constraints, rt_mpc(i) <= rt_impl(i) + M*(1 - pt_mpc{i}(2))];

end

% robustness_margin should be adjusted based on maximum error
% between concrete and reference systems
% (this will depend on the L-infinity gain of the feedback K)

% slack variable is needed since nonlinear system cannot be
% predicted exactly by discrete time model
slack = sdpvar;
constraints = [constraint, slack <= 0];

%%% ADDITIONAL CONSTRAINTS %%%
% these constraints are added to the open_loop_MPC implementation
% resulting in open_loop_star
% which will be called by the closed loop reference controller
for i = 1:H
    constraints = [constraints, rt_mpc(i) >= P(i) + slack];
end

%%% OBJECTIVE_FUNCTION %%%
M = 1e4;
[Q_bar, q_bar, R_bar, r_bar] = make_QP_costs(T,Q,Qf,q,qf,R,r);
obj_fun = 1/2*(x_bar'*Q_bar*x_bar + u_bar'*R_bar*u_bar) + ...
                q_bar'*x_bar + r_bar'*u_bar - M*slack;

%%% CALL SOLVER %%%
optimize(constraints, obj_fun, sdpsettings('solver','gurobi'));
u_opt = value(u_bar);

% output initial condition for next iteration, if needed
xt = value(x{1});

end