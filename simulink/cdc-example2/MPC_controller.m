function MPC_controller(block)

  setup(block);
  
%endfunction

function setup(block)
  %% Register number of input and output ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions = 3;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(2).Dimensions = 2;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions = 2;
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Run accelerator on TLC
  block.SetAccelRunOnTLC(true);
  
  %% Register methods
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
%endfunction

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  
%endfunction

function Output(block)
    
    %%% BLOCK INPUTS %%%
    x1 = block.InputPort(1).data; % initial state
    u0 = block.InputPort(2).data; % initial input
    
    % check if the simulation is initializing
    % if so, set initial input to u0 = [0; 2]
    % this way, the input rate constraint is not violated at the start
    if(u0(2) == 0)
        u0(2) = 2;
    end
    
    % problem parameters
    % samping time
    dt = 0.2;
    % obstacles
%     % center and radius of obstacles
%     num_obs = 3;
%     xc = cell(num_obs,1);
%     radius = cell(num_obs,1);
%     xc{1} = [40; 7]; % right obstacle
%     radius{1} = 3;
%     xc{2} = [10; 9]; % left obstacle
%     radius{2} = 3;
%     xc{3} = [25; 10]; % center obstacle
%     radius{3} = 3;

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
    % slack = 1.44;
    
    %%% DECISION VARIABLES %%%
    T = 30; % time horizon
    n = 3; % state dim
    m = 2; % input dim
    x = cell(T+1,1);
    u = cell(T,1);
    for i = 1:T
        x{i} = sdpvar(n,1);
        u{i} = sdpvar(m,1);
    end
    x{T+1} = sdpvar(n,1);
    slack = sdpvar(1,1);
    
    %%% CONSTRAINTS & COST WEIGHTS %%%
    % input constraints
    Hu = [eye(2); -eye(2)];
    hu = [pi / 8; 4; pi / 8; -2];
    % input rate constraints
    Hr = [eye(2); -eye(2)];
    hr = [0.1 * pi / 5; 0.1 * 0.75; 0.1 * pi / 5; 0.1 * 0.75];
    % cost weights
    Q = (1e-4)*eye(3,3);
    q = zeros(3,1);
    R = (1e-4)*eye(2,2);
    r = zeros(2,1);
    % final state cost
    % Qf = (1e-4)*eye(3,3);
    % final state target set
    ell = 2.5; % half-width of box enclosing desired final state
    x_des = [30; 0; 0] + [ell; -ell; 0];
    Hf = [eye(2), zeros(2,1); -eye(2), zeros(2,1)];
    hf = [x_des(1) + ell;
          x_des(2) + ell;
          -(x_des(1) - ell);
          -(x_des(2) - ell)];
    % input rate penalty
    alpha = 1e-4;
    % tracker error bound (from Galaxy's computations)
    err_bound = 1.44;
    
    % slack variable bound
    slack_bound = 2;
    
    %%% SET UP PROBLEM %%%
    % obj fun
    obj_fun = 0;
    % add constraints
    constraints = (x{1} == x1);
    for i = 1:T
        % obj fun
        obj_fun = obj_fun + (1/2)*((x{i}-x_des)'*Q*(x{i}-x_des) + u{i}'*R*u{i}) + q'*x{i} + r'*u{i};
        % dynamics constraints
        constraints = [constraints, x{i+1} == x{i} + dt*f_car_approx(x{i}, u{i})];
        if(i == 1)
            % input rate constraints (must be hard constraint)
            constraints = [constraints, Hr*(u{1} - u0) <= hr];
            obj_fun = obj_fun + alpha*(u{1} - u0)'*eye(2)*(u{1} - u0);
        else
            % input rate constraints (must be hard constraint)
            constraints = [constraints, Hr*(u{i} - u{i-1}) <= hr];
            obj_fun = obj_fun + alpha*(u{i} - u{i-1})'*eye(2)*(u{i} - u{i-1});
        end
        % state & input constraints (must be hard constraint)
        constraints = [constraints, Hu*u{i} <= hu];
        for j = 1:num_obs
            % obstacle avoidence constraints (use slack variable)
            constraints = [constraints, (x{i+1}(1:2) - xc{j})'*(x{i+1}(1:2) - xc{j}) >= (radius{j} + err_bound + slack_bound - slack)^2];
            constraints = [constraints, slack >= 0, slack_bound >= slack];
        end
    end
    % obj_fun = obj_fun + (1/2)*(x{T+1}-x_des)'*Qf*(x{T+1}-x_des) + slack^2;
    obj_fun = obj_fun + ones(1, length(hf))*max(Hf*x{T+1} - hf, 0) + 100*slack^2;
    
    %%% CALL SOLVER %%%
    diagnostics = optimize(constraints, obj_fun, sdpsettings('solver', 'ipopt'));
    if(diagnostics.problem == 1)
        error('Problem was infeasible!');
    elseif(diagnostics.problem ~= 0)
        disp('Some issue occurred with ipopt');
    end
    
    % implement optimal control input on system
    u_opt = value(u{1});
    block.OutputPort(1).Data = u_opt;
    
%endfunction
