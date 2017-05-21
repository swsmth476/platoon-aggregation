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
  
  zt = block.InputPort(1).Data;
  time = block.InputPort(2).Data;
  
  % in order to fix weird initialization bug
  if(zt == zeros(6,1))
      zt = [mdl.z0; zeros(2,1)]; % set to initial state
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
  j_ub = 0.4;
  j_lb = -0.4;
  
  % input constraints
  Hu = [eye(2); -eye(2)];
  hu = [j_ub; j_ub; -j_lb; -j_lb];
  
  % event trigger time (when the platoon is told to accelerate / increase
  % headway)
  event_trigger = 20; % (begin at 2s)
  
  if(time_step < mdl.mpc_H)
      
      % create acceleration signal
      signal = 2*(1:mdl.mpc_H > event_trigger) - 1;
      
      % initial state
      z0 = [mdl.z0; zeros(2,1)];
      
      % transient phase of MPC
      mdl.mpc_P(time_step + 1) = 0;
      [v_opt, ~] = open_loop_star(A,B,theta,z0,mdl.mpc_H,Q,Qf,q,qf,R,r, ...
                                    Hu,hu,mdl.mpc_P,mdl.ut_old,signal);
      v_idx = (time_step*2 + 1):(time_step*2 + 2);
      delta_v = v_opt(v_idx);
      
      % store old inputs for next iteration
      % conditionals necessary because simulink repeats some initial time steps
      if(size(mdl.ut_old,2) < time_step + 1) % save input if not stored yet
          mdl.ut_old = [mdl.ut_old, delta_v];
      else % if this time step was already saved, replace the old one
          mdl.ut_old(:, time_step + 1) = delta_v;
      end
      
      % if at the end of transiet phase, save MPC starting save
      if(size(mdl.ut_old,2) == mdl.mpc_H)
          mdl.zt = z0;
      end

  else
      
      % create acceleration signal
      signal = 2*((time_step - mdl.mpc_H + 1):(time_step) > event_trigger) - 1;
      
      % initial state
      z0 = mdl.zt;
      
      % stationary phase of MPC
      mdl.mpc_P = zeros(mdl.mpc_H,1);
      % NOTE: can call "open_loop_star2( ... )" to simulate 2nd example
      [v_opt, zt_next] = open_loop_star(A,B,theta,z0,mdl.mpc_H,Q,Qf,q,qf,R,r, ...
                                    Hu,hu,mdl.mpc_P,mdl.ut_old,signal);
      v_idx = (mdl.mpc_H*2 + 1):(mdl.mpc_H*2 + 2);
      delta_v = v_opt(v_idx);

      % store old inputs for next iteration
      old_idx = 3:(mdl.mpc_H*2 + 2);
      mdl.ut_old = v_opt(old_idx);
      mdl.ut_old = reshape(mdl.ut_old, 2, mdl.mpc_H);
      
      % save starting state
      mdl.zt = zt_next;
      
  end
  
  % implement input on system
  v = zt(5:6);
  block.OutputPort(1).Data = v + delta_v;

%endfunction