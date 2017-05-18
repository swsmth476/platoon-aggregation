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
  
%   % input cost factor
%   f = 0;
%   
%   Q = blkdiag(1, 2.5, 1); % quadratic state cost
%   R = f*eye(2); % quadratic input cost
%   
%   % construct standard qp weights
%   H = mdl.Gd'*mdl.H'*Q*mdl.H*mdl.Gd + R;
%   f = ((mdl.Fd*z + mdl.theta_hatd)'*mdl.H' - mdl.wg')*Q*mdl.H*mdl.G;
%   
%   % acceleration input bounds
%   a_upper = 2;
%   a_lower = -3;
%   
%   % velocity bounds
%   v1 = [0 1 0 0];
%   v2 = [0 0 0 1]; % to pull out velocity 1 & 2 from ref state
%   v_upper = 30;
%   v_lower = 5;
%   
%   % headway error tolerance
%   % (contributes to relative error between concrete/reference systems)
%   h_delta = 15;
%   hl = [1 0 -1 0]; % to get lead vehicle's separation
%   hd = 50*3; % desired headway
%   
%   % construct qp constraints
%   A = [1 0;
%       -1 0;
%       0 1;
%       0 -1;
%       v1*mdl.Gd;
%       -v1*mdl.Gd;
%       v2*mdl.Gd;
%       -v2*mdl.Gd;
%       hl*mdl.Gd;
%       -hl*mdl.Gd];
% 
%   b = [a_upper;
%       -a_lower;
%       a_upper;
%       -a_lower;
%       v_upper - v1*(mdl.Fd*z + mdl.theta_hatd);
%       -v_lower + v1*(mdl.Fd*z + mdl.theta_hatd);
%       v_upper - v2*(mdl.Fd*z + mdl.theta_hatd);
%       -v_lower + v2*(mdl.Fd*z + mdl.theta_hatd);
%       h_delta + hd - hl*(mdl.Fd*z + mdl.theta_hatd);
%       h_delta - hd + hl*(mdl.Fd*z + mdl.theta_hatd)];
%  
%  [v, ~, exitflag] = quadprog(H, f, A, b);
%  
%  if(exitflag ~= 1)
%      error(['quadprog failed with exit flag ', num2str(exitflag)]);
%  end
%  
%  block.OutputPort(1).Data = v;

v = z(5:6);

% find time step index and use lookup table to select input
dt = 0.1;
time_step = floor(time/dt);
indices = (time_step*2 + 1):(time_step*2 + 2);
delta_v = mdl.u_opt(indices);

block.OutputPort(1).Data = v + delta_v;

%endfunction