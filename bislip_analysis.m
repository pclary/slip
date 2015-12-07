%% Real-time animation of simulated bislip system
if exist('vis', 'var') && isa(vis, 'BiSLIPGraphics') && vis.isAlive()
    vis.reset();
else
    vis = BiSLIPGraphics();
    timedisp = uicontrol('Style', 'text', 'Parent', vis.Fig);
end
vis.setGround(ground_data);
t = X.Time;
t0 = 0;

rate = 1;

tic;
framecount = 0;
while toc*rate < t(end) && vis.isAlive()
    tt = t0 + toc*rate;
    i = find(t <= tt, 1, 'last');
    XX = X.Data(i, :)';
    body = XX([1 3]);
    angle = XX(5);
    toeA = body + XX(9)*[sin(XX(11) + XX(5)); -cos(XX(11) + XX(5))];
    toeB = body + XX(15)*[sin(XX(17) + XX(5)); -cos(XX(17) + XX(5))];
    vis.setState(body, angle, toeA, toeB);
    
    microsecs = floor((tt - floor(tt))*1000);
    seconds = mod(floor(tt), 60);
    minutes = floor(tt/60);
    timedisp.String = sprintf('%.2d:%.2d.%.3d', minutes, seconds, microsecs);
    drawnow;
    framecount = framecount + 1;
end
framerate = framecount/toc

%% Make gif
if exist('vis', 'var') && isa(vis, 'BiSLIPGraphics') && vis.isAlive()
    vis.reset();
else
    vis = BiSLIPGraphics();
end
t = X.Time;

vis.Axes.XTick = [];
vis.Axes.YTick = [];
vis.Axes.XLabel.String = '';
vis.Axes.YLabel.String = '';
vis.Axes.Title.String = '';

tspan = [0 t(end)];
filename = 'slip.gif';

rate = 1;
framerate = 1/30;

tt = tspan(1);
while tt < tspan(2)
    i = find(t <= tt, 1, 'last');
    XX = X.Data(i, :)';
    body = XX([1 3]);
    angle = XX(5);
    toeA = body + XX(9)*[sin(XX(11) + XX(5)); -cos(XX(11) + XX(5))];
    toeB = body + XX(15)*[sin(XX(17) + XX(5)); -cos(XX(17) + XX(5))];
    vis.setState(body, angle, toeA, toeB);
    vis.setGround(@(x) ground_height_sample(x, ground_data), 100);
    drawnow;
    
    frame = getframe(vis.Axes);
    im = frame2im(frame);
    [A, map] = rgb2ind(im, 256);
    if tt == tspan(1)
        imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', framerate);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', framerate);
    end
    
    tt = tt + framerate*rate;
end
