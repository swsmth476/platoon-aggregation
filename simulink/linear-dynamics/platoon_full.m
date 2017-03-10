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
  
  block.InputPort(1).Dimensions        = 2;
  block.InputPort(1).DirectFeedthrough = false;
  
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
  block.ContStates.Data = % insert initial state;
  
%endfunction

function Output(block)

  block.OutputPort(1).Data = block.ContStates.Data;
  
%endfunction

function Derivative(block)
    global mdl;

    % full platoon dynamics (12 states) %
    
    x = block.InputPort(1).Data;
    u = block.InputPort(2).Data;
    
    block.Derivatives.Data = mdl.A*x + mdl.B*u;
  
%endfunction