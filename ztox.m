function x = ztox(z,p)
% map from abstraction state z to concrete state x

% dh = desired headway
% k = spring gain

dh = 50;
k = 5;

P_sub = [1 0;
    0 1;
    1 0;
    0 1;
    1 0;
    0 1];

P = blkdiag(P_sub,P_sub);

omega = [0;
        0;
        -g_inv(p.v0L1 - p.v01,p.dh,p.k);
        0;
        -g_inv(p.v0L1 - p.v01,p.dh,p.k) - g_inv(p.v0L1 - p.v02,p.dh,p.k);
        0;
        0;
        0;
        -g_inv(p.v0L2 - p.v03,p.dh,p.k);
        0;
        -g_inv(p.v0L2 - p.v03,p.dh,p.k) - g_inv(p.v0L2 - p.v04,p.dh,p.k);
        0];
    
x = repmat(omega, 1, size(z,2)); % offset

x = x + P*z; % lift z coordinates into x-space

end