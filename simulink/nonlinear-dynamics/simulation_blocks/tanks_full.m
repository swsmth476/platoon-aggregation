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
 
  block.InputPort(1).Dimensions        = 4;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 4;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 4;
  
  %% Set block sample time to continuous
  block.SampleTimes = [0 0];
  
  %% Setup Dwork
  block.NumContStates = 4;

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

    % full tank dynamics (4 states) %
    x = block.InputPort(1).Data;
    u = block.InputPort(2).Data;
    
    % dynamics of individual tanks 1 - 4 %
    block.Derivatives.Data(1) = f(mdl.gamma, x(1)) + ...
                                u(1) - 1/mdl.A1*phi(mdl.k1, x(1) - x(2));
    block.Derivatives.Data(2) = f(mdl.gamma, x(2)) + ...
                                u(2) + 1/mdl.A1*phi(mdl.k1, x(1) - x(2)) ...
                                     - 1/mdl.A2*phi(mdl.k2, x(2) - x(3));
    block.Derivatives.Data(3) = f(mdl.gamma, x(3)) + ...
                                u(3) + 1/mdl.A2*phi(mdl.k2, x(2) - x(3)) ...
                                     - 1/mdl.A3*phi(mdl.k3, x(3) - x(4));
    block.Derivatives.Data(4) = f(mdl.gamma, x(4)) + ...
                                u(4) + 1/mdl.A3*phi(mdl.k3, x(3) - x(4));

%endfunction