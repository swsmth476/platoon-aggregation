% create figure
figure('Renderer', 'painters', 'Position', [10 10 900 375]);

% center and radius of obstacles
num_obs = 4;
xc = cell(num_obs,1);
radius = cell(num_obs,1);
xc{1} = [-5; -2.5]; % bottom left obstacle
radius{1} = 3;
xc{2} = [12.5; 10]; % top left obstacle
radius{2} = 3;
xc{3} = [30; 7.5]; % top right obstacle
radius{3} = 3;
xc{4} = [15; -15]; % bottom right obstacle
radius{4} = 3;
% slack variable bound
slack = 1.44;

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
% spacing = 0.085;
% subaxis(1,2,1,'Spacing', spacing);
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
    patch(x{i}, y{i}, 'y', 'FaceColor', [0.9290 0.6940 0.1250]);
end
for i = 1:num_obs
    xc_3D{i} = [xc{i}; 0];
    h{i} = ellipse(radius{i}, radius{i}, 0, xc{i}(1), xc{i}(2));
    x{i} = get(h{i}, 'Xdata');
    y{i} = get(h{i}, 'Ydata');
    hold on;
    patch(x{i}, y{i}, 'y', 'FaceColor', [0.8500 0.3250 0.0980]);
end

% plot trajectory & goalset
x_goal = [30 30 35 35];
y_goal = [0 -5 -5 0];
fill(x_goal,y_goal,[0.4660 0.6740 0.1880]);
% goalSet = Polyhedron(Hf, hf);
% plot(goalSet, 'Color', [0.4660 0.6740 0.1880]);
hold on;
planner = plot(xhat_t(:,1), xhat_t(:,2), '-', 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
hold on;
tracker = plot(x_t(:,1), x_t(:,2), '--', 'LineWidth', 1.5, 'Color', [0.4940 0.1840 0.5560]);
hold on;
plot(x_init(1), x_init(2), 'x', 'LineWidth', 3.5, 'Color', [0.6350 0.0780 0.1840]);
legend([planner, tracker],'Planner', 'Tracker', ...
        'Location', 'northoutside', 'Orientation', 'horizontal');

% adjust width & height of plot, add title & labels
xlim([-15 40]);
xticks([-10 0 10 20 30 40]);
xlabel('X Coordinate (m)');
ylim([-20 20]);
yticks([-20 -10 0 10 20]);
ylabel('Y Coordinate (m)');
grid on;
% title('Planner Tracker Framework: Motion Planning Example');
% make tikz file
matlab2tikz('veh_obs_avoid_traj.tex', 'height', '5.5cm', 'width', '6.9cm');

% values taken from the paper
max_err_norm = norm([1.07; 1.44; 1.05]);

% plot error bound
% subaxis(1,2,2,'Spacing', spacing);
figure();
plot(t_sim, vecnorm(err_t')', '-', 'LineWidth', 1.35);
hold on;
plot(t_sim, max_err_norm .* ones(length(err_t)), '--', 'LineWidth', 1.35, ...
        'Color', [0.9290 0.6940 0.1250]);
legend('$\|e(t)\|$', 'Error Bound', 'interpreter', 'latex', ...
        'Location', 'northoutside', 'Orientation', 'horizontal');

% adjust width & height of plot, add title & labels
xlim([0 14]);
xticks([0 2 4 6 8 10 12 14]);
xlabel('Time (s)');
ylim([0 2.5]);
yticks([0 1 2]);
% ylabel('$\|e(t)\|$','interpreter','latex');
grid on;

% make tikz file
matlab2tikz('veh_obs_avoid_err.tex', 'height', '5.5cm', 'width', '6.9cm');
