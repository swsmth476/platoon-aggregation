function bicycle_model(block)

  setup(block);
  
%endfunction

function setup(block)
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 2;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 6;
  
  %% Set block sample time to continuous
  block.SampleTimes = [0 0];
  
  %% Setup Dwork
  block.NumContStates = 6;

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Derivatives',             @Derivative);  
  
%endfunction

function InitConditions(block)

    % initialize leader state
    block.ContStates.Data = [0; 15; 0; 0; 2; 0];

%endfunction

function Output(block)

    block.OutputPort(1).Data = block.ContStates.Data;

%endfunction

function Derivative(block)

    % state & input
    x = block.ContStates.Data;
    u = block.InputPort(1).Data;
    
    % final state target set
    ell = 2.5; % half-width of box enclosing desired final state
    x_des = [30; 0; 0; 0; 0; 0] + [ell; -ell; 0; 0; 0; 0];
    Hf = [eye(2), zeros(2,4); -eye(2), zeros(2,4)];
    hf = [x_des(1) + ell;
          x_des(2) + ell;
          -(x_des(1) - ell);
          -(x_des(2) - ell)];
    % check if the vehicle has reached the target set
    if(sum(Hf*x <= hf) == 4)
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

    % compute time derivative
    block.Derivatives.Data = f_bicycle_v2(x, u);

%endfunction
