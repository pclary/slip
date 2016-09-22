function biped_playback(varargin)

% Input parsing
p = inputParser;
p.addOptional('rate', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addOptional('t0', 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addOptional('filename', '', @ischar);
p.parse(varargin{:});

rate = p.Results.rate;
t0 = p.Results.t0;

vis = BipedVisualization();
vis.env = evalin('base', 'env');
vis.ground_data = evalin('base', 'ground_data');
vis.setup(RobotState());
timedisp = uicontrol('Style', 'text', 'Parent', vis.getFig());

X = evalin('base', 'X');
Time = X.body.x.Time;

tic;
framecount = 0;
while toc*rate < Time(end) && vis.isAlive()
    t = t0 + toc*rate;
    i = find(Time <= t, 1, 'last');
    
    Xi = RobotState();
    Xi.body.x = X.body.x.data(i);
    Xi.body.y = X.body.y.data(i);
    Xi.body.theta = X.body.theta.data(i);
    Xi.right.l = X.right.l.data(i);
    Xi.right.l_eq = X.right.l_eq.data(i);
    Xi.right.theta = X.right.theta.data(i);
    Xi.right.theta_eq = X.right.theta_eq.data(i);
    Xi.left.l = X.left.l.data(i);
    Xi.left.l_eq = X.left.l_eq.data(i);
    Xi.left.theta = X.left.theta.data(i);
    Xi.left.theta_eq = X.left.theta_eq.data(i);
    
    vis.step(Xi);
    
    microsecs = floor((t - floor(t))*1000);
    seconds = mod(floor(t), 60);
    minutes = floor(t/60);
    timedisp.String = sprintf('%.2d:%.2d.%.3d', minutes, seconds, microsecs);
    drawnow;
    framecount = framecount + 1;
end
fprintf('Frame rate: %.1f fps\n', framecount/toc);
