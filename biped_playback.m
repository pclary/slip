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
timedisp = uicontrol('Style', 'text', 'Parent', vis.Fig);

X = evalin('base', 'X');

tic;
framecount = 0;
while toc*rate < X.Time(end) && vis.isAlive()
    t = t0 + toc*rate;
    i_X = find(X.Time <= t, 1, 'last');
    vis.step(X.Data(i_X, :));
    
    microsecs = floor((t - floor(t))*1000);
    seconds = mod(floor(t), 60);
    minutes = floor(t/60);
    timedisp.String = sprintf('%.2d:%.2d.%.3d', minutes, seconds, microsecs);
    drawnow;
    framecount = framecount + 1;
end
fprintf('Frame rate: %.1f fps\n', framecount/toc);
