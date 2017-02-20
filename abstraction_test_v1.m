syms xL vL x1 v1 x2 v2 v0L v01 v02 k1 k2 d1 d2;

% real state = [xL vL x1 v1 x2 v2]^T
% abstraction state = [xL vL]^T

% constant distance headway case

A = [0 1 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 1 0 0;
    k1 0 -k1 -1 0 0;
    0 0 0 0 0 1;
    0 0 k2 0 -k2 -1];

theta = [0; 0; 0; v01 - k1*d1; 0; v02 - k2*d2];

P = [1 0;
    0 1;
    1, -1/k1;
    0 1;
    1, -1/k1-1/k2;
    0 1];

omega = [0; 0; -d1 + v01/k1; 0; -d1 - d2 + v01/k1 + v02/k2; 0];

% abstraction

A_hat = [0 1;
        0 0];

theta_hat = [0; 0];

% P = [1 0;
%     0 1;
%     1 0;
%     0 1;
%     1 0;
%     0 1];
% 
% omega = [0; 0; -d; 0; -2*d; 0];
% 
% theta = [0; v0L; 0; v01 + k*d; 0; v02 + k*d];
% 
% A_hat = [0 1; 0 -1];
% 
% theta_hat = [0; v0L];