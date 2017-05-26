function concrete_controller(block)

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 3;
  block.NumOutputPorts = 2;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
  
  block.InputPort(1).Dimensions        = 12;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 4;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.InputPort(3).Dimensions        = 2;
  block.InputPort(3).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 12; 
  block.OutputPort(2).Dimensions       = 2;
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Run accelerator on TLC
  block.SetAccelRunOnTLC(true);
  
  %% Register methods
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  block.RegBlockMethod('InitializeConditions',     @InitConditions);  
  block.RegBlockMethod('Outputs',                  @Output);  
  
%endfunction

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  block.OutputPort(2).SamplingMode  = fd;
  
%endfunction

function InitConditions(block) 
  block.OutputPort(1).Data = zeros(12,1);
  block.OutputPort(2).Data = [0; 0];
  
%endfunction

function Output(block)
  global mdl;
  
  x = block.InputPort(1).Data;
  z = block.InputPort(2).Data;
  v = block.InputPort(3).Data;

  e = x - mdl.P*z - mdl.omega; % relative error
  block.OutputPort(1).Data = e;
  block.OutputPort(2).Data = mdl.R*v + mdl.K*e;
  
%endfunction