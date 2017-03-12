function reference_controller(block)

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of dialog parameters   
  block.NumDialogPrms = 0;

  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 4;
  block.InputPort(1).DirectFeedthrough = false;

  block.OutputPort(1).Dimensions       = 2;
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  
%endfunction

function InitConditions(block) 
  block.OutputPort(1).Data = [0; 0];
  
%endfunction

function Output(block)
  global mdl;

  % model predictive controller for reference model %
  
  z = block.InputPort(1).Data;
  
  Q = eye(3); % quadratic state cost
  R = eye(2); % quadratic input cost
  
  % construct standard qp weights
  H = mdl.Gd'*mdl.H'*Q*mdl.H*mdl.Gd + R;
  f = ((mdl.Fd*z + mdl.theta_hatd)'*mdl.H' - mdl.wg')*Q*mdl.H*mdl.G;
  
  % input bounds
  v_upper = 2;
  v_lower = -3;
  
  % headway error tolerance
  % (contributes to relative error between concrete/reference systems)
  h_delta = 15;
  
  % construct qp constraints
  A = [1 0;
      -1 0;
      0 1;
      0 -1;
      mdl.d*mdl.Gd;
      -mdl.d*mdl.Gd];
      
  b = [v_upper;
      -v_lower;
      v_upper;
      -v_lower;
      h_delta + mdl.e + mdl.d*(mdl.Fd*z + mdl.theta_hatd);
      h_delta + mdl.e + mdl.d*(mdl.Fd*z + mdl.theta_hatd)];
  
  [v, ~, exitflag] = quadprog(H, f, A, b);
  
  if(exitflag ~= 1)
      error(['quadprog failed with exit flag ', num2str(exitflag)]);
  end
  
  block.OutputPort(1).Data = v;

%endfunction