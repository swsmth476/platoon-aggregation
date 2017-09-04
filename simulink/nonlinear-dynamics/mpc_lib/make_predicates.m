function [num_pred, mu_a, mu_b, num_phi, num_psi] = make_predicates
%%% Summary %%%
% Make predicates which the STL formula 1 is comprised of

%%% Description %%%
%
% STL formula predicates %
% mu1: tank 12 water level upper bound (safety)
% mu2: tank 12 water level lower bound (safety)
% mu3: tank 34 water level upper bound (safety)
% mu4: tank 34 water level lower bound (safety)
% mu5: flux 12 upper bound (safety)
% mu6: flux 12 lower bound (safety)
% mu7: flux 34 upper bound (safety)
% mu8: flux 34 lower bound (safety)
% mu9: tank 12 water level lower bound (target)
% mu10: tank 34 water level upper bound (target)

% number of predicates in the formula
num_pred = 10;

% STL formula values %
safety_ub = 9;
safety_lb = 1;
target_ub = 4;
target_lb = 6;
flux_bound = 10;

% introduce predicate variables
% predicates are affine, of the form mu(x(i)) = a*x(i) + b %

% row 'j' represents 'a_j' from predicate mu_i(x(i))= a_j*x(i) + b_j
mu_a = [-1 0 0 0;
        1 0 0 0;
        0 -1 0 0;
        0 1 0 0;
        0 0 -1 0;
        0 0 1 0;
        0 0 0 -1;
        0 0 0 1;
        1 0 0 0;
        0 -1 0 0];

% row 'j' represents 'b_j' from predicate mu_i(x_t) = a_j*x(i) + b_j
mu_b = [safety_ub;
        -safety_lb;
        safety_ub;
        -safety_lb;
        flux_bound;
        flux_bound;
        flux_bound;
        flux_bound;
        -target_lb;
        target_ub];
    
% number of conjunctions for each variable
num_phi = 8;
num_psi = 2;

end

