function dx = f_bicycle_v2(x, u)
%%% Description %%%
%
% Dynamic Bicycle Model
%
% States are [x;        (position)
%             y;        (position)
%             psi;      (yaw)
%             psi_dot;  (yaw rate)
%             X;        (longitudinal speed)
%             Y];       (lateral speed) 
%
% Inputs are [front wheel steering angle; longitudinal acceleration]
%

% load constants into a struct
con.m = 1.67e3; % mass (kg)
con.I_z = 2.1e3; % yaw inertia (kg/m^2)
con.l_f = 0.99; % distance from vehicle center of mass to front axle (m)
con.l_r = 1.7; % distance from vehicle center of mass to rear axle (m)
con.C_af = -6.1595e4; % front tire cornering stiffness (N/rad)
con.C_ar = -5.2095e4; % rear tire cornering stiffness (N/rad)

% compute tire slip angle for the front and rear wheels
alpha_f = (x(6) + con.l_f * x(4)) * inverse_approx(x(5)) - u(1);
alpha_r = (x(6) - con.l_r * x(4)) * inverse_approx(x(5));

% compute lateral tire force at front and rear wheels
F_cf = con.C_af * alpha_f;
F_cr = con.C_ar * alpha_r;

% equations of motion
dx = zeros(6,1);
dx(1) = x(5) * cos(x(3)) - x(6) * sin(x(3));
dx(2) = x(5) * sin(x(3)) + x(6) * cos(x(3));
dx(3) = x(4);
dx(4) = 2 / con.I_z * (con.l_f * F_cf - con.l_r * F_cr);
dx(5) = x(4) * x(6) + u(2);
dx(6) = -x(4) * x(5) + 2 / con.m * (F_cf + F_cr);

function y = inverse_approx(x)
    y = -0.1201 * x + 0.7082;
    % y = 1 / x;
end

end