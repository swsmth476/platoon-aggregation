function [e, xl] = get_error(z,tz,x,tx,p)
% compute error between abstraction and concrete trajectories
% z is abstraction system trajectory
% tz is set of time points for z
% x is concrete system trajectory
% tx is set of time points for x

xl = ztox(z,p); % lift z coordinates to x-space

xli = [interp1(tz,xl(1,:),tx)';
        interp1(tz,xl(2,:),tx)';
        interp1(tz,xl(3,:),tx)';
        interp1(tz,xl(4,:),tx)';
        interp1(tz,xl(5,:),tx)';
        interp1(tz,xl(6,:),tx)';
        interp1(tz,xl(7,:),tx)';
        interp1(tz,xl(8,:),tx)';
        interp1(tz,xl(9,:),tx)';
        interp1(tz,xl(10,:),tx)';
        interp1(tz,xl(11,:),tx)';
        interp1(tz,xl(12,:),tx)']; % interpolate at tx time points
    
e = zeros(1,size(xli,2));
        
for i = 1:size(xli,2)
    e(i) = norm(x(:,i) - xli(:,i));
end

end