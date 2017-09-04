function reference_controller(block)

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
 
  block.InputPort(1).Dimensions        = 6;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = false;

  block.OutputPort(1).Dimensions       = 2;
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('InitializeConditions',     @InitConditions);  
  block.RegBlockMethod('Outputs',                  @Output);
  
%endfunction

function InitConditions(block) 
  block.OutputPort(1).Data = [0; 0];
  
%endfunction

function Output(block)
  global mdl;

  % model predictive controller for reference model %
  
  z = block.InputPort(1).Data;
  time = block.InputPort(2).Data;
  
  % in order to fix weird initialization bug
  if(z == zeros(6,1))
      z = [mdl.z0; zeros(2,1)]; % set to initial state
  end
  
  v = z(5:6);
  
  % find time step index and use lookup table to select input
  dt = 0.1;
  time_step = floor(time/dt);
  indices = (time_step*2 + 1):(time_step*2 + 2);
  delta_v = mdl.u_opt(indices);
  
  % in order to prevent input jumps
  if(time_step == 0 && mdl.init == 0)
    mdl.init = 1;
  else 
    block.OutputPort(1).Data = v + delta_v;
  end

%endfunction