function platoon_full(block)

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of dialog parameters   
  block.NumDialogPrms = 0;

  %% Register number of input and output ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 12;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 2;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 12;
  
  %% Set block sample time to continuous
  block.SampleTimes = [0 0];
  
  %% Setup Dwork
  block.NumContStates = 12;

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Derivatives',             @Derivative);  
  
%endfunction

function InitConditions(block)

  %% Initialize Dwork
  global mdl;
  
  block.ContStates.Data = mdl.x0;
  
%endfunction

function Output(block)

  block.OutputPort(1).Data = block.ContStates.Data;
  
%endfunction

function Derivative(block)
    global mdl;

    % full platoon dynamics (12 states) %
    
    x = block.InputPort(1).Data;
    u = block.InputPort(2).Data;
    
    % full dynamics with air drag
    dx = zeros(8,1);
    dx(1) = x(2); % position a
    dx(2) = mdl.v_0 - x(2) - mdl.rho*x(2)^2/mdl.m + u(1); % velocity a
    dx(3) = x(4); % position 1
    dx(4) = mdl.v_0 - x(4) - mdl.rho*x(4)^2/mdl.m + mdl.k*(x(2) - x(4) - mdl.dh); % velocity 1
    dx(5) = x(6); % position b
    dx(6) = mdl.v_0 - x(6) - mdl.rho*x(6)^2/mdl.m + u(2) + mdl.k*(x(4) - x(6) - mdl.dh); % velocity b
    dx(7) = x(8); % position 2
    dx(8) = mdl.v_0 - x(8) - mdl.rho*x(8)^2/mdl.m + mdl.k*(x(6) - x(8) - mdl.dh); % velocity 2
    
    block.Derivatives.Data = dx;
  
%endfunction