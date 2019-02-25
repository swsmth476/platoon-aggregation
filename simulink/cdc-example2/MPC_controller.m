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
    dt = 0.1;
    % input rate constraints
    vel_rate = 0.25;
    turn_rate = pi/50;
    % obstacles
    obs = cell(2,1);
    A1 = [eye(2); -eye(2)];
    b1 = [25; 50; 0; -25];
    A2 = [eye(2); -eye(2)];
    b2 = [45; 15; -15; 0];
    obs{1} = Polyhedron('A', A1, 'b', b1);
    obs{2} = Polyhedron('A', A2, 'b', b2);
    
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
    
    %%% CONSTRAINTS & COST WEIGHTS %%%
    % state constraints
    Hx = [eye(3); -eye(3)];
    hx = [60; 60; 15; 0; 0; 0];
    Hxf = Hx;
    hxf = hx;
    % input constraints
    Hu = [eye(2); -eye(2)];
    hu = [15; 2*pi/5; 0; 0];
    % input rate constraints
    Hr = [eye(2); -eye(2)];
    hr = [15/100; 2*pi/10; -15/100; -2*pi/10];
    % cost weights
    Q = zeros(3,3);
    q = zeros(3,1);
    R = (1e-4)*eye(2,2);
    r = zeros(2,1);
    % final state cost
    x_des = [50; 0; 0];
    Qf = diag([1; 1; 0]);
    qf = -x_des;
    
    %%% SET UP PROBLEM %%%
    % obj fun
    obj_fun = 0;
    % slack variable
    slack = sdpvar;
    % add constraints
    constraints = (x{1} == x1);
    cosntraints = [constraints, slack >= 0];
    for i = 1:T
        % obj fun
        obj_fun = obj_fun + (1/2)*(x{i}'*Q*x{i} + u{i}'*R*u{i}) + q'*x{i} + r'*u{i};
        % dynamics constraints
        constraints = [constraints, x{i+1} == x{i} + dt*f_car_approx(x{i}, u{i})];
        if(i == 1)
            % input rate constraints (must be hard constraint)
            constraints = [constraints, max(Hr*(u{1} - u0) - hr) - slack <= 0];
        else
            % input rate constraints (must be hard constraint)
            constraints = [constraints, max(Hr*(u{i} - u{i-1}) - hr) - slack <= 0];
        end
        % state & input constraints
        constraints = [constraints, max(Hx*x{i} - hx) - slack <= 0]; % add slack variable
        constraints = [constraints, max(Hu*u{i} - hu) - slack <= 0];
        % obstacle avoidence constraints
        for j = 1:length(obs)
            % A*x <= b representation of obstacles
            A = obs{j}.A;
            b = obs{j}.b;
            % assuming each obstacle has 4 sides
            % constraints = [constraints, ( A(1,:)*x{j}(1:2) >= b(1,:) ...
            %                            | A(2,:)*x{j}(1:2) >= b(2,:) ...
            %                            | A(3,:)*x{j}(1:2) >= b(3,:) ...
            %                            | A(4,:)*x{j}(1:2) >= b(4,:) ) ];
        end
    end
    constraints = [constraints, Hxf*x{T+1} <= hxf];
    obj_fun = obj_fun + (1/2)*x{T+1}'*Qf*x{T+1} + qf'*x{T+1} + 100*slack^2;
    
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
