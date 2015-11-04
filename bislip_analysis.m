%% Real-time animation of simulated bislip system
if exist('vis', 'var') && isa(vis, 'BiSLIPGraphics') && vis.isAlive()
    vis.clearTrace();
else
    vis = BiSLIPGraphics();
end
t = X.Time;

rate = 0.1;

tic;
while toc*rate < t(end)
    tt = toc*rate;
    i = find(t <= tt, 1, 'last');
    XX = X.Data(i, :)';
    body = XX([1 3]);
    angle = XX(5);
    toeA = body + XX(9)*[sin(XX(11) + XX(5)); -cos(XX(11) + XX(5))];
    toeB = body + XX(15)*[sin(XX(17) + XX(5)); -cos(XX(17) + XX(5))];
    vis.setState(body, angle, toeA, toeB);
    vis.setGround(@(x) ground_height_sample(x, ground_data), 100);
    drawnow;
end

%% Make gif
if exist('vis', 'var') && isa(vis, 'BiSLIPGraphics') && vis.isAlive()
    vis.clearTrace();
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
