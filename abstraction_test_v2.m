syms xL vL x1 v1 x2 v2 v0L v01 v02 k1 k2;

% real state = [xL vL x1 v1 x2 v2]^T
% abstraction state = [xL vL]^T

% constant time headway case

A = [0 1 0 0 0 0;
    0 -1 0 0 0 0;
    0 0 0 1 0 0;
    k1 0 -k1, -1-2*k1 0 0;
    0 0 0 0 0 1;
    0 0 k2 0 -k2, -1-2*k2];

theta = [0; v0L; 0; v01; 0; v02];

P = [1 0;
    0 1;
    1, -1/k1 - 2;
    0 1;
    1, -1/k2 - 2;
    0 1];

omega = [0; 0; v01/k1; 0; v02/k2; 0]; % <= distances added must be cumulative

% abstraction

A_hat = [0 1;
        0 -1];
    
theta_hat = [0; v0L];