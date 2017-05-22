function [num_pred, mu_a, mu_b, num_phi, num_psi] = make_predicates2
%%% Summary %%%
% Make predicates which the STL formula 2 is comprised of

%%% Description %%%
%
% STL formula predicates %
% mu1: vehicle 1 velocity lower bound
% mu2: vehicle 1 velocity upper bound
% mu3: vehicle 2 velocity lower bound
% mu4: vehicle 2 velocity upper bound
% mu5: vehicle 1 accel upper bound
% mu6: vehicle 1 accel lower bound
% mu7: vehicle 2 accel upper bound
% mu8: vehicle 2 accel lower bound
% mu9: platoon target headway lower bound
% mu10: platoon target headway upper bound

% number of predicates in the formula
num_pred = 10;

% STL formula values %
headway_des = 200;
headway_delta = 10;
vel_des = 25;
vel_delta = 2.5;

% state bounds
headway_lb = headway_des - headway_delta;
headway_ub = headway_des + headway_delta;
vel_lb = vel_des - vel_delta;
vel_ub = vel_des + vel_delta;
accel_bd = 8; % absolute value |accel| < accel_bd

% introduce predicate variables
% predicates are affine, of the form mu(x(i)) = a*x(i) + b %

% row 'j' represents 'a_j' from predicate mu_i(x(i))= a_j*x(i) + b_j
mu_a = [0 1 0 0 0 0;
        0 -1 0 0 0 0;
        0 0 0 1 0 0;
        0 0 0 -1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 -1 0;
        0 0 0 0 0 1;
        0 0 0 0 0 -1;
        1 0 -1 0 0 0;
        -1 0 1 0 0 0];

% row 'j' represents 'b_j' from predicate mu_i(x_t) = a_j*x(i) + b_j
mu_b = [-vel_lb;
        vel_ub;
        -vel_lb;
        vel_ub;
        accel_bd;
        accel_bd;
        accel_bd;
        accel_bd;
        -headway_lb;
        headway_ub];
    
% number of conjunctions for each variable
num_phi = 8;
num_psi = 2;

end

