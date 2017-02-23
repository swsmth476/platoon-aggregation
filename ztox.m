function x = ztox(z, v0, v01, v02)
% map from abstraction state z to concrete state x

% dh = desired headway
% k = spring gain

dh = 50;
k = 5;

P = [1 0;
    0 1;
    1 0;
    0 1;
    1 0;
    0 1];

omega = [0;
        0;
        0;
        -g_inv(v0 - v01,dh,k);
        0;
        -g_inv(v0 - v01,dh,k)-g_inv(v0 - v02,dh,k)];
    
x = blkdiag(P,P)*z + [omega; omega];

end