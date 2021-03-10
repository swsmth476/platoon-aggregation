%
figure('Renderer', 'painters', 'Position', [10 10 900 375]);

% center and radius of obstacles
num_obs = 3;
xc = cell(num_obs,1);
radius = cell(num_obs,1);
xc{1} = [40; 7]; % right obstacle
radius{1} = 3;
xc{2} = [10; 9]; % left obstacle
radius{2} = 3;
xc{3} = [25; 5]; % center obstacle
radius{3} = 3;
% slack variable bound
slack = 2;

% starting point, goal point
xstart = [0; 15; 0];
xfinish = [50; 0; 0];

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

% plot start / end points, trajectory
plot(xstart(1), xstart(2), 'x', 'LineWidth', 3.5);
plot(xfinish(1), xfinish(2), 'x', 'LineWidth', 3.5);
plot(xhat(:,1), xhat(:,2), 'o');

% adjust width & height of plot, add title & labels
xlim([-5, 55]);
xlabel('X Coordinate');
ylim([-5, 20]);
ylabel('Y Coordinate');
title('3D Dubins Vehicle Path');