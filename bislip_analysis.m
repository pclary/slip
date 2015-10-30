%% Real-time animation of simulated bislip system
tic;
if exist('vis', 'var') && isa(vis, 'BiSLIPGraphics') && vis.isAlive()
    vis.clearTrace();
else
    vis = BiSLIPGraphics();
end
t = X.Time;

rate = 1;

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
