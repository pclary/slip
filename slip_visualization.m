% adapted from http://blogs.mathworks.com/graphics/2014/10/21/double_pendulum/
function slip_visualization(block)

setup(block)


function setup(block)

block.NumInputPorts  = 1;
block.NumOutputPorts = 0;

block.SetPreCompInpPortInfoToDynamic;

block.InputPort(1).Dimensions = 9;

block.RegBlockMethod('Start',   @Start);
block.RegBlockMethod('Outputs', @Output);

block.SetSimViewingDevice(true);


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
end
