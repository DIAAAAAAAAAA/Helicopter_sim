function sf_heli_in(block)
% sf_heli_in, a level-2 s-function for reading data the helicopter
%
% Fankai Zhang 17-04-2013

setup(block);

function setup(block)

% Register the number of ports.
block.NumInputPorts  = 0;
block.NumOutputPorts = 1;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override the output port properties.
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode  = 'Sample';
block.OutputPort(1).Dimensions = 4;

% Register the parameters.
block.NumDialogPrms = 1;

% Register the sample times.
block.SampleTimes = [0 0];

% -----------------------------------------------------------------
% Options
% -----------------------------------------------------------------
% Specify if Accelerator should use TLC or call back to the
% M-file
block.SetAccelRunOnTLC(false);

% -----------------------------------------------------------------
% Register methods called at run-time
% -----------------------------------------------------------------

block.RegBlockMethod('ProcessParameters', @ProcessPrms);
block.RegBlockMethod('Outputs', @Outputs);


function ProcessPrms(block)

block.AutoUpdateRuntimePrms;


function Outputs(block)

H = block.DialogPrm(1).Data;

if ~isempty(H)
    ys = helicopter('read', H);
else
    ys = zeros(1,4);
end
block.OutputPort(1).Data = ys;