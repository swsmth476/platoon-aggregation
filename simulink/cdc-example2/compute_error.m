function compute_error(block)

  setup(block);
  
%endfunction

function setup(block)
  %% Register number of input and output ports
  block.NumInputPorts  = 3;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions = 6;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(2).Dimensions = 2;
  block.InputPort(2).DirectFeedthrough = false;
  block.InputPort(3).Dimensions = 3;
  block.InputPort(3).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions = 6;
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Run accelerator on TLC
  block.SetAccelRunOnTLC(true);
  
  %% Register methods
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
%endfunction

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  
%endfunction

function Output(block)

    %%% BLOCK INPUTS %%%
    x = block.InputPort(1).data; % state
    uhat = block.InputPort(2).data; % abstract input
    xhat = block.InputPort(3).data; % abstract state
    
    if(norm(x - zeros(6,1)) < 0.1)
        x = [0; 15; 0; 0; 0; 0];
    end
    
    % define error manifold
    pi_val = [xhat; uhat; 0];
    
    % redefine error state as described in paper
    R = [cos(xhat(3)), -sin(xhat(3));
         sin(xhat(3)),  cos(xhat(3))];
    phi = blkdiag(inv(R), eye(4));
    err = phi * (x - pi_val);

    block.OutputPort(1).Data = err;

%endfunction
