function a = g_fn(distance, desired_headway, gain)
% nonlinear function that comprises the spring between vehicles
% input distance = distance between vehicles
% desired_headway = desired inter-vehicle spacing
% gain = gain of spring
% output a = resulting acceleration adjustment

a = gain*log(distance/desired_headway);

end