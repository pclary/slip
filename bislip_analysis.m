%% Real-time animation of simulated bislip system
tic;
if exist('vis', 'var') && isa(vis, 'BiSLIPGraphics') && vis.isAlive()
    vis.clearTrace();
else
    vis = BiSLIPGraphics();
end
t = X.Time;

rate = 0.01;

while toc*rate < t(end)
    tt = toc*rate;
    i = find(t <= tt, 1, 'last');
    vis.setState(X.Data(i, 1:2), X.Data(i, 5), X.Data(i, 7:8), X.Data(i, 12:13));
    vis.setGround(@(x) ground_height_sample(x, ground_data), 100);
    drawnow;
end
