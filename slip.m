function slip(block)

setup(block);


function setup(block)

% Register number of ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 2;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
% Angle Control
block.InputPort(1).Dimensions = 1;
block.InputPort(1).DatatypeID = 0;  % double
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Length control
block.InputPort(2).Dimensions = 1;
block.InputPort(2).DatatypeID = 0;  % double
block.InputPort(2).Complexity = 'Real';
block.InputPort(2).DirectFeedthrough = true;

% Override output port properties
% [com_x, com_y, com_xdot, com_ydot, angle, length_eq, length_comp, grf_x, grf_y]
block.OutputPort(1).Dimensions = 9;
block.OutputPort(1).DatatypeID = 0; % double
block.OutputPort(1).Complexity = 'Real';

% SLIP Phase
block.OutputPort(2).Dimensions = 1;
block.OutputPort(2).DatatypeID = 6;
block.OutputPort(2).Complexity = 'Real';

% Register parameters
% 1: Mass
% 2: Stiffness
% 3: Damping
% 4: Gravity
% 5: Initial state [x, y, xdot, ydot]
block.NumDialogPrms = 5;
block.DialogPrmsTunable = {'Tunable', 'Tunable', 'Tunable', 'Tunable', 'Nontunable'};

% Register sample times
block.SampleTimes = [0 0]; % inherited

% Specify the block simStateCompliance
block.SimStateCompliance = 'DefaultSimState';

% Continuous states
block.NumContStates = 4;

% Register methods
block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortSamplingMode);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate);


function InitializeConditions(block)

block.ContStates.Data = block.DialogPrm(5).Data;


function DoPostPropSetup(block)

block.NumDworks = 2;

% SLIP Phase
block.Dwork(1).Name            = 'Phase';
block.Dwork(1).Dimensions      = 1;
block.Dwork(1).DatatypeID      = 6;
block.Dwork(1).Complexity      = 'Real';
block.Dwork(1).UsedAsDiscState = true;

% Toe position
block.Dwork(2).Name            = 'Toe';
block.Dwork(2).Dimensions      = 2;
block.Dwork(2).DatatypeID      = 0; % double
block.Dwork(2).Complexity      = 'Real';
block.Dwork(2).UsedAsDiscState = false;


function SetInputPortSamplingMode(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  block.OutputPort(2).SamplingMode  = fd;


function Start(block)

block.Dwork(1).Data = int32(SlipPhase.Flight);
block.Dwork(2).Data = [0 0];


function Outputs(block)

switch block.Dwork(1).Data
    case SlipPhase.Flight
        angle = block.InputPort(1).Data;
        length_eq = block.InputPort(2).Data;
        length_comp = length_eq;
        grf_x = 0;
        grf_y = 0;
    case SlipPhase.Stance
        leg = block.Dwork(2).Data - block.ContStates.Data(1:2);
        angle = atan2(leg(1), -leg(2));
        length_eq = block.InputPort(2).Data;
        length_comp = norm(leg);
        dY = slip_stance(block.ContStates.Data, block.DialogPrm(1).Data, ...
            block.DialogPrm(2).Data, block.DialogPrm(3).Data, ...
            block.DialogPrm(4).Data, block.InputPort(2).Data, ...
            block.Dwork(2).Data);
        grf_x = block.DialogPrm(1).Data*dY(3);
        grf_y = block.DialogPrm(1).Data*(dY(4) + block.DialogPrm(4).Data);
    otherwise
        angle = 0;
        length_eq = 0;
        length_comp = 0;
        grf_x = 0;
        grf_y = 0;
end

block.OutputPort(1).Data = [block.ContStates.Data' angle length_eq length_comp grf_x grf_y];
block.OutputPort(2).Data = block.Dwork(1).Data;


function Update(block)

switch block.Dwork(1).Data
    case SlipPhase.Flight
        th = block.InputPort(1).Data;
        l = block.InputPort(2).Data;
        leg = l*[sin(th) -cos(th)];
        toe = block.ContStates.Data(1:2)' + leg;
        if toe(2) <= ground_height(toe(1))
            block.Dwork(1).Data = int32(SlipPhase.Stance);
            block.Dwork(2).Data = toe;
        end
    case SlipPhase.Stance
        leg = block.Dwork(2).Data - block.ContStates.Data(1:2);
        if norm(leg) > block.InputPort(2).Data
            block.Dwork(1).Data = int32(SlipPhase.Flight);
        end
    otherwise
        % error
end


function Derivatives(block)

switch block.Dwork(1).Data
    case SlipPhase.Flight
        dY = slip_flight(block.ContStates.Data, block.DialogPrm(4).Data);
    case SlipPhase.Stance
        dY = slip_stance(block.ContStates.Data, block.DialogPrm(1).Data, ...
            block.DialogPrm(2).Data, block.DialogPrm(3).Data, ...
            block.DialogPrm(4).Data, block.InputPort(2).Data, ...
            block.Dwork(2).Data);
    otherwise
        dY = [0 0 0 0]';
end

if ground_height(block.ContStates.Data(1)) >= block.ContStates.Data(2)
    dY = [0 0 0 0]';
end

block.Derivatives.Data = dY;


function Terminate(block)



function dY = slip_flight(Y, g)

dY = [Y(3:4); 0; -g];


function dY = slip_stance(Y, m, k, b, g, l, toe)

leg = Y(1:2) - toe;
lc = norm(leg);
dlc = dot(Y(3:4), leg)/l;
F = -(leg/lc)*(k*(lc - l) + b*dlc);
dY = [Y(3:4); F/m + [0; -g]];

