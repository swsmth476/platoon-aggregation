function parameters = set_parameters
% set all necessary parameters for platoon simulation

% nominal velocities
parameters.v0L1 = 32; % lead vehicle 1 (similarly named for others)
parameters.v0L2 = 30; 
parameters.v01 = 25; % ^follower vehicle 1 (same for others)
parameters.v02 = 25;
parameters.v03 = 25;
parameters.v04 = 25;

parameters.dh = 50; % desired headway
parameters.k = 5; % gain for spring

end

