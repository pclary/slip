function bislip_sfun(block)

setup(block);


function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Control inputs: [length_eq_a; length_eq_b; torque_a; torque_b]
block.InputPort(1).Dimensions = 4;
block.InputPort(1).DatatypeID = 0;  % double
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).DirectFeedthrough = false;


% States: [body_x; body_y; body_th; com_xdot; body_ydot; body_thdot; 
%          foot_a_x; foot_a_y; foot_a_xdot; foot_a_ydot; 
%          foot_b_x; foot_b_y; foot_b_xdot; foot_b_ydot]
block.OutputPort(1).Dimensions = 14;
block.OutputPort(1).DatatypeID = 0; % double
block.OutputPort(1).Complexity = 'Real';

% Register parameters
% 1: Initial states
% 2: Body mass
% 3: Body inertia
% 4: Foot mass
% 5: Leg stiffness
% 6: Leg damping
% 7: Gravity
% 8: Ground data
block.NumDialogPrms = 8;
block.DialogPrmsTunable = {'Nontunable', 'Tunable', 'Tunable', 'Tunable', ...
    'Tunable', 'Tunable', 'Tunable', 'Nontunable'};

% Register sample times
block.SampleTimes = [0 0]; % inherited

% Specify the block simStateCompliance
block.SimStateCompliance = 'DefaultSimState';

% Continuous states
block.NumContStates = 14;

% Register methods
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Outputs', @Outputs);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate);


function InitializeConditions(block)

block.ContStates.Data = block.DialogPrm(1).Data;


function Outputs(block)

block.OutputPort(1).Data = block.ContStates.Data;


function Derivatives(block)

if block.CurrentTime > 1
    0;
end

block.Derivatives.Data = bislip_dynamics(block.ContStates.Data, block.InputPort(1).Data, ...
    [block.DialogPrm(2).Data; block.DialogPrm(3).Data; block.DialogPrm(4).Data; ...
     block.DialogPrm(5).Data; block.DialogPrm(6).Data; block.DialogPrm(7).Data], ...
     block.DialogPrm(8).Data);


function Terminate(block)


