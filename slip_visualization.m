% adapted from http://blogs.mathworks.com/graphics/2014/10/21/double_pendulum/
function slip_visualization(block)

setup(block)


function setup(block)

block.NumInputPorts  = 2;
block.NumOutputPorts = 0;

block.SetPreCompInpPortInfoToDynamic;

block.InputPort(1).Dimensions = 9;
block.InputPort(1).DatatypeID = 0;  % double
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).Dimensions = 1;
block.InputPort(2).DatatypeID = 6;  % int32
block.InputPort(2).Complexity = 'Real';
block.InputPort(2).DirectFeedthrough = true;

block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('Start',                @Start);
block.RegBlockMethod('Outputs',              @Output);
block.RegBlockMethod('Update',               @Update);

block.SetSimViewingDevice(true);


function DoPostPropSetup(block)

block.NumDworks = 2;
block.Dwork(1).Name       = 'PrevPhase';
block.Dwork(1).Dimensions = 1;
block.Dwork(1).DatatypeID = 6;
block.Dwork(1).Complexity = 'Real';

block.Dwork(2).Name       = 'TDEvent';
block.Dwork(2).Dimensions = 1;
block.Dwork(2).DatatypeID = 8;
block.Dwork(2).Complexity = 'Real';


function Start(block)

ud = get_param(block.BlockHandle, 'UserData');
if isempty(ud)
    vis = [];
else
    vis = ud.vis;
end

if isempty(vis) || ~isa(vis, 'SlipGraphics') || ~vis.isAlive()
    vis = SlipGraphics();
else
    vis.clearTrace();
end

ud.vis = vis;
set_param(block.BlockHandle, 'UserData', ud);


function Output(block)

if block.IsMajorTimeStep
    
    ud = get_param(block.BlockHandle, 'UserData');
    vis = ud.vis;
    
    if isempty(vis) || ~isa(vis, 'SlipGraphics') || ~vis.isAlive()
        return;
    end
    
    body = block.InputPort(1).Data(1:2);
    th = block.InputPort(1).Data(5);
    toe = body + block.InputPort(1).Data(7)*[sin(th); -cos(th)];
    
    vis.setState(body, toe);
    vis.setGround(@ground_height, 100);
    
    if block.Dwork(2).Data
        vis.addStep(toe);
    end
end


function Update(block)

block.Dwork(2).Data = (block.Dwork(1).Data ~= block.InputPort(2).Data) && ...
    (block.InputPort(2).Data == int32(SlipPhase.Stance));
block.Dwork(1).Data = block.InputPort(2).Data;

