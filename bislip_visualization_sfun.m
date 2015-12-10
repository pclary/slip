% adapted from http://blogs.mathworks.com/graphics/2014/10/21/double_pendulum/
function bislip_visualization_sfun(block)

setup(block)


function setup(block)

block.NumInputPorts  = 2;
block.NumOutputPorts = 1;

block.SetPreCompInpPortInfoToDynamic;

block.InputPort(1).Dimensions = 18;
block.InputPort(1).DatatypeID = 0;  % double
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).Dimensions = 4;
block.InputPort(2).DatatypeID = 0;  % double
block.InputPort(2).Complexity = 'Real';
block.InputPort(2).DirectFeedthrough = true;

block.OutputPort(1).Dimensions = 2;
block.OutputPort(1).DatatypeID = 0;  % double
block.OutputPort(1).Complexity = 'Real';

block.NumDialogPrms = 1;

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

if isempty(vis) || ~isa(vis, 'BiSLIPGraphics') || ~vis.isAlive()
    vis = BiSLIPGraphics();
else
    vis.reset();
end

vis.setGround(block.DialogPrm(1).Data);

ud.vis = vis;
set_param(block.BlockHandle, 'UserData', ud);


function Output(block)

if block.IsMajorTimeStep
    
    ud = get_param(block.BlockHandle, 'UserData');
    vis = ud.vis;
    
    if isempty(vis) || ~isa(vis, 'BiSLIPGraphics') || ~vis.isAlive()
        return;
    end
    
    vis.setState(block.InputPort(1).Data, block.InputPort(2).Data);
    
    if vis.dragEnabled()
        x = vis.DragLine.XData;
        y = vis.DragLine.YData;
        v = [x(2) - x(1); y(2) - y(1)];
    else
        v = [0; 0];
    end
    block.OutputPort(1).Data = v;
end
