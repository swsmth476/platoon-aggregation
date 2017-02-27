% Dynamics for a single platoon (NLPlatoonFull simulates 2 such platoons)
% d/dt xL = vL
% d/dt vL = -vL + v0 + u1(t)
% d/dt x1 = v1
% d/dt v1 = -v1 + v01 + g(xL - x1)
% d/dt x2 = v2
% d/dt v2 -v2 + v02 + g(x1 - x2)

% let g(x) = unidirectional spring = log(x)

x0 = [300; 0; 250; 0; 200; 0; 150; 0; 100; 0; 50; 0];
z0 = [300; 0; 150; 0]; % initial conditions

parameters = set_parameters; % set parameters for platoon

% simulate concrete platoon
[tx,x] = ode45(@(t1,x) NLPlatoonFull(t1,x,parameters), [0, 80], x0);
x = x';

% simulate abstraction platoon
[tz,z] = ode45(@(t2,z) NLPlatoonAbs(t2,z,parameters), [0, 80], z0);
z = z';

% get e(t) = norm(x(t) - (Pz(t) + omega))
% e is the distance from x to the invariant manifold
[e, xl] = get_error(z,tz,x,tx,parameters);

input2 = zeros(size(tx));

% post-compute inputs (fix this later)
for i = 1:length(tx)
    input_u1(i) = u1(tx(i));
    input_u2(i) = u2(tx(i),x(5,i) - x(7,i), parameters);
    input_v1(i) = v1(tx(i));
    input_v2(i) = v2(tx(i));
end

position_indices = [1; 3; 5; 7; 9; 11];

% plot results
subplot(2,2,1);
plot(tx,x(position_indices,:));
title('x(t)');
xlabel('Time (s)');
ylabel('Distance (m)');

subplot(2,2,2);
plot(tz,xl(position_indices,:));
title('Pz(t) - \Omega');
xlabel('Time (s)');
ylabel('Distance (m)');

subplot(2,2,3);
plot(tx,input_u2,tx,input_v2);
title('u_2(t) & v_2(t)');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('u_2(t)','v_2(t)');

subplot(2,2,4);
plot(tx,e);
title('|x(t) - Pz(t) + \Omega|_2');
xlabel('Time (s)');
ylabel('State Space Distance');