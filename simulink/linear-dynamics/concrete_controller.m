function concrete_controller(block)

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
  
  block.InputPort(1).Dimensions        = 12;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 4;
  block.InputPort(2).DirectFeedthrough = false;
  
  %   block.InputPort(2).Dimensions        = 2;
  %   block.InputPort(2).DirectFeedthrough = true;
  
  block.OutputPort(1).Dimensions       = 2;
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Run accelerator on TLC
  block.SetAccelRunOnTLC(true);
  
  %% Register methods
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  
%endfunction

function InitConditions(block) 
  block.OutputPort(1).Data = 0;
  
%endfunction

function Output(block)
  global mdl;
  
  x = block.InputPort(1).Data;
  z = block.InputPort(2).Data;
  % v = block.InputPort(3).Data;
  
  % possibly change later to include R & Q terms as in Girardi paper

  block.OutputPort(1).Data = mdl.K*(mdl.P*z + mdl.omega - x);
  
%endfunction