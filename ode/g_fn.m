function a = g_fn(d,dh,k)
% nonlinear function that comprises the spring between vehicles
% d = distance between vehicles
% dh = desired headway (inter-vehicle spacing)
% k = gain of spring
% output a = resulting acceleration adjustment

a = k*log(d/dh);

end