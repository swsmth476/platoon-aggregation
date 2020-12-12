function concrete_controller_v3(block)

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
  
  block.InputPort(1).Dimensions        = 8;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 4;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.InputPort(3).Dimensions        = 2;
  block.InputPort(3).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 8; 
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
  block.OutputPort(1).Data = zeros(8,1);
  block.OutputPort(2).Data = [0; 0];
  
%endfunction

function Output(block)
  global mdl;
  
  x = block.InputPort(1).Data;
  z = block.InputPort(2).Data;
  v = block.InputPort(3).Data;
  
  if(isnan(v(1)) || isnan(v(2)))
    % to get rid of simulation initialization error
    v = zeros(2,1);
  end

  % relative error
  e = x - mdl.P*z - mdl.omega;
  
%   % to match Galaxy's convention
%   err_1 = e(1);
%   err_2 = e(2);
%   err_3 = e(3);
%   err_4 = e(4);
%   err_5 = e(5);
%   err_6 = e(6);
%   err_7 = e(7);
%   err_8 = e(8);
%   
%   % feedback laws
%   k1 = 7.310237077417656*err_1^2 + 0.8793221137430846*err_1*err_2 ...
%   - 2.653249943015845*err_1*err_3 + 4.331517094161538*err_1*err_4 ...
%   - 9.405241393436171*err_1*err_5 - 2.337070300891735*err_1*err_6 ...
%   + 2.465564481003121*err_1*err_7 - 1.775874756470845*err_1*err_8 ...
%   + 0.106858450141636*err_2^2 - 0.6036252904700181*err_2*err_3 ...
%   + 0.2809270678224076*err_2*err_4 - 0.7149754923791132*err_2*err_5 ...
%   - 0.452293142050096*err_2*err_6 + 0.29902044471224*err_2*err_7 ...
%   - 0.1503608749659174*err_2*err_8 + 0.7984481775361802*err_3^2 ...
%   - 0.5685353067004819*err_3*err_4 + 1.185511930987479*err_3*err_5 ...
%   + 0.1359104639139365*err_3*err_6 - 0.6375867327385107*err_3*err_7 ...
%   + 0.07089129516714007*err_3*err_8 + 0.8038833534312299*err_4^2 ...
%   - 3.136981751694757*err_4*err_5 - 0.5964645721608571*err_4*err_6 ...
%   + 0.8313952321493777*err_4*err_7 - 0.670198178157845*err_4*err_8 ...
%   + 4.314863044022124*err_5^2 + 1.213643272474482*err_5*err_6 ...
%   - 3.297606009544968*err_5*err_7 + 1.201044066563762*err_5*err_8 ...
%   - 0.02380653677946781*err_6^2 - 0.4227062305771899*err_6*err_7 ...
%   + 0.0260550250100964*err_6*err_8 + 1.046308924871968*err_7^2 ...
%   - 0.2646368889828776*err_7*err_8 + 0.1575902604484178*err_8^2 ...
%   - 39.00973925264029*err_1 - 8.997646026991644*err_2 + 7.166736275969146*err_3 ...
%   - 7.434038210184606*err_4 + 13.07289205051026*err_5 ...
%   + 1.730687964112148*err_6 - 0.7807557872756278*err_7 ...
%   + 0.3281937383832421*err_8;
% 
%   k2 = -12.04333495124612*err_1^2 + 4.809191429636601*err_1*err_2 ...
%   - 1.165437677326205*err_1*err_3 - 4.385274205769068*err_1*err_4 ...
%   + 9.415703648259019*err_1*err_5 + 1.531017144711677*err_1*err_6 ...
%   + 1.437312326407256*err_1*err_7 + 1.016703729031732*err_1*err_8 ...
%   + 0.3036327594187764*err_2^2 - 0.2216101202185274*err_2*err_3 ...
%   + 0.9865956195312605*err_2*err_4 - 2.513057447619145*err_2*err_5 ...
%   + 0.04968627463267571*err_2*err_6 + 1.140996174827358*err_2*err_7 ...
%   + 0.273077652593042*err_2*err_8 - 2.71028625941595*err_3^2 ...
%   + 0.02414324853771274*err_3*err_4 + 0.9790412680064849*err_3*err_5 ...
%   + 1.718456780054533*err_3*err_6 - 1.278139964176268*err_3*err_7 ...
%   - 1.828368802470845*err_3*err_8 - 1.378980278051424*err_4^2 ...
%   + 4.691864707296751*err_4*err_5 + 1.566010570336735*err_4*err_6 ...
%   - 1.131953443734133*err_4*err_7 + 0.8372251969780619*err_4*err_8 ...
%   - 13.16439041494156*err_5^2 - 3.263159747791836*err_5*err_6 ...
%   + 11.68193941503532*err_5*err_7 - 3.500628146352885*err_5*err_8 ...
%   + 0.08735809757542279*err_6^2 - 0.5344832237510823*err_6*err_7 ...
%   - 0.9963983993522596*err_6*err_8 - 4.27715855890616*err_7^2 ...
%   + 1.399857885532207*err_7*err_8 - 1.168813345405126*err_8^2 ...
%   + 22.52870007833811*err_1 - 0.749375059346054*err_2 - 27.16444613421267*err_3 ...
%   + 5.563853951216391*err_4 - 49.8496703444007*err_5 ...
%   - 26.1721099240233*err_6 + 33.36147063853657*err_7 - 2.963878206202456*err_8;

  k1 = 0;
  k2 = 0;
  
  block.OutputPort(1).Data = e;
  block.OutputPort(2).Data = double([k1; k2]) + v;
  
%endfunction