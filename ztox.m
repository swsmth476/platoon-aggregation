function x = ztox(z, v0, v01, v02)
% map from abstraction state z to concrete state x

P = [1 0;
    0 1;
    1 0;
    0 1;
    1 0;
    0 1];

omega = [0;
        0;
        0;
        -g_inv(v0 - v01);
        0;
        -g_inv(v0 - v01)-g_inv(v0 - v02)];
    
x = blkdiag(P,P)*z + [omega; omega];

end