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
  
  % find time step index
  dt = 0.1;
  time_step = floor(time/dt);
  
  % create augmented system dynamics
  A = [[mdl.Fd mdl.Gd]; zeros(2,4) eye(2)];
  B = [zeros(4,2); eye(2)];
  theta = [mdl.theta_hatd; zeros(2,1)];
  
  % choose augmented system state/input costs
  Q = zeros(6);
  Qf = zeros(6);
  q = zeros(6,1);
  qf = zeros(6,1);
  R = eye(2);
  r = zeros(2,1);
  
  % jerk constraints
  j_ub = 0.2;
  j_lb = -0.2;
  
  % input constraints
  Hu = [eye(2); -eye(2)];
  hu = [j_ub; j_ub; -j_lb; -j_lb];
  
  % create acceleration signal 
  signal = ((time_step):(time_step+mdl.mpc_H) > 15);
  time_step = time_step + 1;
  
  if(time_step < mdl.mpc_H)
      % transient phase of MPC
      x0 = [mdl.z0; zeros(2,1)];
      mdl.mpc_P(time_step) = 0;
      v_opt = open_loop_star1(A,B,theta,x0,mdl.mpc_H,Q,Qf,q,qf,R,r, ...
                                    Hu,hu,mdl.mpc_P,mdl.ut_old,signal);
      indices = (time_step*2 + 1):(time_step*2 + 2);
      delta_v = v_opt(indices);
      mdl.ut_old(:,time_step) = delta_v;
%   else
%       % stationary phase of MPC
%       v_opt = open_loop_star1(A,B,theta,z,mdl.STL_H,Q,Qf,q,qf,R,r, ...
%                                 Hu,hu,zeros(mdl.STL_H,1),ut_old);
%       
  end
  
  % implement input
  v = z(5:6);
  block.OutputPort(1).Data = v + delta_v;

%endfunction