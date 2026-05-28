function sf_heli_out(block)
% sf_heli_out, a level-2 s-function for writing data the helicopter
%
% Fankai Zhang 17-04-2013


setup(block);

function setup(block)

% Register the number of ports.
block.NumInputPorts  = 1;
block.NumOutputPorts = 0;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

block.InputPort(1).DatatypeID  = 0; % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).SamplingMode  = 'Sample';
block.InputPort(1).Dimensions = 2;


% Register the parameters.
block.NumDialogPrms = 2;
block.SampleTimes = [block.DialogPrm(2).Data, 0];

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
block.RegBlockMethod('Terminate', @Terminate);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

function ProcessPrms(block)

block.AutoUpdateRuntimePrms;

function InitializeConditions(block)
block.Dwork(1).Data = 0;
tic;

function DoPostPropSetup(block)
block.NumDworks = 1;

block.Dwork(1).Name            = 'SampleCounter';
block.Dwork(1).Dimensions      = 1;
block.Dwork(1).DatatypeID      = 0;      % double
block.Dwork(1).Complexity      = 'Real'; % real

function Outputs(block)

H = block.DialogPrm(1).Data;
Ts = block.DialogPrm(2).Data;

if ~isempty(H)
    ua = block.InputPort(1).Data;
    helicopter('write', H, ua);                     % send control input to process
end

n = block.Dwork(1).Data + 1;
while toc < n*Ts; end;
block.Dwork(1).Data = n;


function Terminate(block)
H = block.DialogPrm(1).Data;
if isempty(H)
    return;
end
helicopter('write', H, [0.0 0.0]);              % reset actuator
helicopter('CloseAll');



