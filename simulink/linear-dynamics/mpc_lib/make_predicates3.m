function [num_pred, mu_a, mu_b, num_phi, num_psi] = make_predicates3
%%% Summary %%%
% Make predicates which the STL formula 3 is comprised of

%%% Description %%%
%
% STL formula predicates %
% mu1: vehicle 1 headway lower bound
% mu2: vehicle 1 headway upper bound
% mu3: vehicle 2 headway lower bound
% mu4: vehicle 2 headway upper bound
% mu5: vehicle 1 velocity lower bound
% mu6: vehicle 1 velocity upper bound
% mu7: vehicle 2 velocity lower bound
% mu8: vehicle 2 velocity upper bound
% mu9: vehicle 1 accel upper bound
% mu10: vehicle 1 accel lower bound
% mu11: vehicle 2 accel upper bound
% mu12: vehicle 2 accel lower bound
% mu13: platoon target headway lower bound
% mu14: platoon target headway upper bound
% mu15: vehicle 1 target speed lower bound
% mu16: vehicle 1 target speed upper bound
% mu17: vehicle 2 target speed lower bound
% mu18: vehicle 2 target speed upper bound

% number of predicates in the formula
num_pred = 11;

% STL formula values %
headway_nominal = 24;
headway_bound = 4; % safety range
headway_des = 24;
headway_delta = 2; % target range
vel_nominal = 28;
vel_bound = 4; % safety range
% vel_des = 28;
% vel_delta = 1; % target range

% state bounds
headway_safe_lb = headway_nominal - headway_bound;
% headway_safe_ub = headway_nominal + headway_bound;
headway_lb = headway_des - headway_delta;
headway_ub = headway_des + headway_delta;
vel_safe_lb = vel_nominal - vel_bound;
vel_safe_ub = vel_nominal + vel_bound;
% vel_lb = vel_des - vel_delta;
% vel_ub = vel_des + vel_delta;
accel_safe_bd = 4; % absolute value |accel| < accel_bd

% introduce predicate variables
% predicates are affine, of the form mu(x(i)) = a*x(i) + b %

% row 'j' represents 'a_j' from predicate mu_i(x(i))= a_j*x(i) + b_j
mu_a = [1 0 -1 0 0 0; % headway bounds
        0 1 0 0 0 0;
        0 -1 0 0 0 0;
        0 0 0 1 0 0;
        0 0 0 -1 0 0; % velocity bounds ^
        0 0 0 0 1 0;
        0 0 0 0 -1 0;
        0 0 0 0 0 1;
        0 0 0 0 0 -1; % accel bounds ^
        1 0 -1 0 0 0;
        -1 0 1 0 0 0]; % target headway ^
        % 0 1 0 0 0 0;
        % 0 -1 0 0 0 0;
        % 0 0 0 1 0 0;
        % 0 0 0 -1 0 0]; % target velocity ^

% row 'j' represents 'b_j' from predicate mu_i(x_t) = a_j*x(i) + b_j
mu_b = [-headway_safe_lb;
        -vel_safe_lb;
        vel_safe_ub;
        -vel_safe_lb;
        vel_safe_ub;
        accel_safe_bd;
        accel_safe_bd;
        accel_safe_bd;
        accel_safe_bd;
        -headway_lb;
        headway_ub];
        % -vel_lb;
        % vel_ub;
        % -vel_lb;
        % vel_ub];

% number of conjunctions for each variable
num_phi = 9;
num_psi = 2;

end
