%% Real-time animation of simulated bislip system
tic;
if exist('vis', 'var') && isa(vis, 'BiSLIPGraphics') && vis.isAlive()
    vis.clearTrace();
else
    vis = BiSLIPGraphics();
end
t = Y.Time;

while toc < t(end)
    tt = toc;
    i = find(t <= tt, 1, 'last');
    vis.setState(Y.Data(i, 1:2), Y.Data(i, 5), Y.Data(i, 7:8), Y.Data(i, 11:12));
    vis.setGround(@(x) ground_height_sample(x, ground_data), 100);
    drawnow;
end
