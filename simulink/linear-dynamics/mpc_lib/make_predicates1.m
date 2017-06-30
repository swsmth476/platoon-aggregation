function [num_pred, mu_a, mu_b, num_phi, num_psi] = make_predicates1
%%% Summary %%%
% Make predicates which the STL formula 1 is comprised of

%%% Description %%%
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

% number of predicates in the formula
num_pred = 10;

% STL formula values %
headway_des = 150;
headway_delta = .5;
vel_des = 28;
vel_delta = 1;

% state bounds
headway_lb = headway_des - headway_delta;
headway_ub = headway_des + headway_delta;
vel_lb = vel_des - vel_delta;
vel_ub = vel_des + vel_delta;
accel_bd = 3.5; % absolute value |accel| < accel_bd

% introduce predicate variables
% predicates are affine, of the form mu(x(i)) = a*x(i) + b %

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
    
% number of conjunctions for each variable
num_phi = 6;
num_psi = 4;

end

