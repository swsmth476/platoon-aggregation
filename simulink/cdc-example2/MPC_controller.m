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
    
    % problem parameters
    % samping time
    dt = 0.2;
    % obstacles
    % center and radius of obstacles
    num_obs = 3;
    xc = cell(num_obs,1);
    radius = cell(num_obs,1);
    xc{1} = [40; 7]; % right obstacle
    radius{1} = 5;
    xc{2} = [10; 9]; % left obstacle
    radius{2} = 5;
    xc{3} = [25; 5]; % center obstacle
    radius{3} = 5;
    
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
    hu = [15; 2*pi/5; 0; 2*pi/5];
    % input rate constraints
    Hr = [eye(2); -eye(2)];
    hr = [0.75; pi/5; 0.75; pi/5];
    % cost weights
    Q = (1e-4)*eye(3,3);
    q = zeros(3,1);
    R = (1e-4)*eye(2,2);
    r = zeros(2,1);
    % final state cost
    x_des = [50; 0; 0];
    Qf = (1e-4)*eye(3,3);
    % input rate penalty
    alpha = 1e-4;
    
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
            constraints = [constraints, (x{i+1}(1:2) - xc{j})'*(x{i+1}(1:2) - xc{j}) >= (radius{j} - slack)^2];
            constraints = [constraints, slack >= 0, 2 >= slack];
        end
    end
    obj_fun = obj_fun + (1/2)*(x{T+1}-x_des)'*Qf*(x{T+1}-x_des) + slack^2;
    
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
