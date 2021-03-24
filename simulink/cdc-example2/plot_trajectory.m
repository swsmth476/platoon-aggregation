% create figure
figure('Renderer', 'painters', 'Position', [10 10 900 375]);

% center and radius of obstacles
num_obs = 3;
xc = cell(num_obs,1);
radius = cell(num_obs,1);
xc{1} = [40; 7]; % right obstacle
radius{1} = 3;
xc{2} = [10; 9]; % left obstacle
radius{2} = 3;
xc{3} = [25; 10]; % center obstacle
radius{3} = 3;
% slack variable bound
slack = 2;

% starting point, goal point
ell = 2.5; % half-width of box enclosing desired final state
x_init = [0; 15; 0];
x_des = [30; 0; 0] + [ell; -ell; 0];
Hf = [eye(2), zeros(2,1); -eye(2), zeros(2,1); zeros(2,2), [1; -1]];
hf = [x_des(1) + ell;
      x_des(2) + ell;
      -(x_des(1) - ell);
      -(x_des(2) - ell);
      0;
      0];

% plot obstacles
xc_3D = cell(num_obs, 1);
h = cell(num_obs, 1);
x = cell(num_obs, 1);
y = cell(num_obs, 1);
for i = 1:num_obs
    xc_3D{i} = [xc{i}; 0];
    h{i} = ellipse(radius{i} + slack, radius{i} + slack, 0, xc{i}(1), xc{i}(2));
    x{i} = get(h{i}, 'Xdata');
    y{i} = get(h{i}, 'Ydata');
    hold on;
    patch(x{i}, y{i}, 'y', 'FaceColor', 'yellow');
end
for i = 1:num_obs
    xc_3D{i} = [xc{i}; 0];
    h{i} = ellipse(radius{i}, radius{i}, 0, xc{i}(1), xc{i}(2));
    x{i} = get(h{i}, 'Xdata');
    y{i} = get(h{i}, 'Ydata');
    hold on;
    patch(x{i}, y{i}, 'y', 'FaceColor', 'red');
end

% plot trajectory & goalset
plot(x_init(1), x_init(2), 'x', 'LineWidth', 3.5);
goalSet = Polyhedron(Hf, hf);
plot(goalSet);
planner = plot(xhat_t(:,1), xhat_t(:,2), 'o');
tracker = plot(x_t(:,1), x_t(:,2), 'x');
legend([planner, tracker],'Planner', 'Tracker');

% adjust width & height of plot, add title & labels
xlim([-5, 55]);
xlabel('X Coordinate');
ylim([-5, 20]);
ylabel('Y Coordinate');
title('Planner Tracker Framework: Motion Planning Example');