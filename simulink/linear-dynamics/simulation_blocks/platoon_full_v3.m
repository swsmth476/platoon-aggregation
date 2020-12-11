function platoon_full_v3(block)

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
 
  block.InputPort(1).Dimensions        = 20;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 2;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 20;
  
  %% Set block sample time to continuous
  block.SampleTimes = [0 0];
  
  %% Setup Dwork
  block.NumContStates = 20;

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
    
%     % full dynamics with air drag
%     dx = zeros(8,1);
%     dx(1) = x(2); % position a
%     dx(2) = mdl.v_0 - x(2) - mdl.rho*x(2)^2/mdl.m + u(1); % velocity a
%     dx(3) = x(4); % position 1
%     dx(4) = mdl.v_0 - x(4) - mdl.rho*x(4)^2/mdl.m + mdl.k*(x(1) - x(3) - mdl.tau*x(2)); % velocity 1
%     dx(5) = x(6); % position b
%     dx(6) = mdl.v_0 - x(6) - mdl.rho*x(6)^2/mdl.m + u(2) + mdl.k*(x(3) - x(5) - mdl.tau*x(4)); % velocity b
%     dx(7) = x(8); % position 2
%     dx(8) = mdl.v_0 - x(8) - mdl.rho*x(8)^2/mdl.m + mdl.k*(x(5) - x(7) - mdl.tau*x(6)); % velocity 2

    % full dynamics with air drag
    dx = zeros(20,1);
    dx(1) = x(2); % position a
    dx(2) = mdl.v_0 - x(2) - mdl.rho*x(2)^2/mdl.m + u(1); % velocity a
    dx(3) = x(4); % position 1
    dx(4) = mdl.v_0 - x(4) - mdl.rho*x(4)^2/mdl.m + mdl.k*(x(1) - x(3) - mdl.dh); % velocity 1
    dx(5) = x(6); % position 2
    dx(6) = mdl.v_0 - x(6) - mdl.rho*x(6)^2/mdl.m + mdl.k*(x(3) - x(5) - mdl.dh); % velocity 2
    dx(7) = x(8); % position 3
    dx(8) = mdl.v_0 - x(8) - mdl.rho*x(8)^2/mdl.m + mdl.k*(x(5) - x(7) - mdl.dh); % velocity 3
    dx(9) = x(4); % position 4
    dx(10) = mdl.v_0 - x(10) - mdl.rho*x(10)^2/mdl.m + mdl.k*(x(7) - x(9) - mdl.dh); % velocity 4
    dx(11) = x(6); % position b
    dx(12) = mdl.v_0 - x(12) - mdl.rho*x(12)^2/mdl.m + u(2) + mdl.k*(x(9) - x(11) - mdl.dh); % velocity b
    dx(13) = x(8); % position 5
    dx(14) = mdl.v_0 - x(14) - mdl.rho*x(14)^2/mdl.m + mdl.k*(x(11) - x(13) - mdl.dh); % velocity 5
    dx(15) = x(4); % position 6
    dx(16) = mdl.v_0 - x(16) - mdl.rho*x(16)^2/mdl.m + mdl.k*(x(13) - x(15) - mdl.dh); % velocity 6
    dx(17) = x(6); % position 7
    dx(18) = mdl.v_0 - x(18) - mdl.rho*x(18)^2/mdl.m + mdl.k*(x(15) - x(17) - mdl.dh); % velocity 7
    dx(19) = x(8); % position 8
    dx(20) = mdl.v_0 - x(20) - mdl.rho*x(20)^2/mdl.m + mdl.k*(x(17) - x(19) - mdl.dh); % velocity 8
    
    block.Derivatives.Data = dx;
  
%endfunction