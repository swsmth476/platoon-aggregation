function platoon_ref_v2(block)

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
  
  block.InputPort(2).Dimensions        = 2;
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
  
  block.ContStates.Data = mdl.z0;
  
%endfunction

function Output(block)

  block.OutputPort(1).Data = block.ContStates.Data;
  
%endfunction

function Derivative(block)
    global mdl;

    % reference platoon dynamics (4 states) %
    
    z = block.InputPort(1).Data;
    v = block.InputPort(2).Data;
    
    % reference dynamics with air drag term
    dz = zeros(4,1);
    dz(1) = z(2); % position a
    dz(2) = mdl.v_0 - z(2) - mdl.rho*z(2)^2/mdl.m + v(1); % velocity a
    dz(3) = z(4); % position b
    dz(4) = mdl.v_0 - z(4) - mdl.rho*z(4)^2/mdl.m + v(2); % velocity b

    block.Derivatives.Data = dz;
  
%endfunction