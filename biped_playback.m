function biped_playback(varargin)

% Input parsing
p = inputParser;
p.addOptional('rate', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addOptional('t0', 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addOptional('filename', '', @ischar);
p.parse(varargin{:});

rate = p.Results.rate;
t0 = p.Results.t0;

framerate = 30;

if ~evalin('base', 'exist(''vis'', ''var'')') || ...
        ~evalin('base', 'strcmp(class(vis), ''BipedVisualization'')') || ...
        ~evalin('base', 'vis.isAlive')
    evalin('base', 'vis = BipedVisualization()')
    evalin('base', 'vis.setup(RobotState())');
end
vis = evalin('base', 'vis');
vis.env = evalin('base', 'env');
vis.ground_data = evalin('base', 'ground_data');
vis.resetPartial();
timedisp = uicontrol('Style', 'text', 'Parent', vis.getFig());

X = evalin('base', 'X');
Time = X.body.x.Time;

tic;
framecount = 0;
t = t0;
while t < Time(end) && vis.isAlive()
    if p.Results.filename
        t = t0 + framecount*rate/framerate;
    else
        t = t0 + toc*rate;
    end
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
    
    if p.Results.filename
        [imind, cm] = rgb2ind(frame2im(getframe(vis.getFig)), 256);
        if framecount == 0
            imwrite(imind, cm, p.Results.filename, 'gif', 'DelayTime', 1/framerate);
        else
            imwrite(imind, cm, p.Results.filename, 'gif', 'DelayTime', 1/framerate, 'WriteMode', 'append');
        end
    end
    
    microsecs = floor((t - floor(t))*1000);
    seconds = mod(floor(t), 60);
    minutes = floor(t/60);
    timedisp.String = sprintf('%.2d:%.2d.%.3d', minutes, seconds, microsecs);
    drawnow;
    framecount = framecount + 1;
end
fprintf('Frame rate: %.1f fps\n', framecount/toc);
