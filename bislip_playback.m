function bislip_playback(varargin)

% Input parsing
p = inputParser;
p.addOptional('rate', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addOptional('t0', 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addOptional('filename', '', @ischar);
p.parse(varargin{:});

rate = p.Results.rate;
t0 = p.Results.t0;

if ~evalin('base', 'exist(''vis'', ''var'') && isa(vis, ''BiSLIPGraphics'') && vis.isAlive()')
    evalin('base', 'vis = BiSLIPGraphics();');
end

vis = evalin('base', 'vis');
vis.reset();
timedisp = uicontrol('Style', 'text', 'Parent', vis.Fig);
    
vis.setGround(evalin('base', 'ground_data'));
X = evalin('base', 'X');
leg_targets = evalin('base', 'leg_targets');

tic;
framecount = 0;
while toc*rate < X.Time(end) && vis.isAlive()
    t = t0 + toc*rate;
    i_X = find(X.Time <= t, 1, 'last');
    i_lt = find(leg_targets.Time <= t, 1, 'last');
    vis.setState(X.Data(i_X, :), leg_targets.Data(:, :, i_lt));
    
    microsecs = floor((t - floor(t))*1000);
    seconds = mod(floor(t), 60);
    minutes = floor(t/60);
    timedisp.String = sprintf('%.2d:%.2d.%.3d', minutes, seconds, microsecs);
    drawnow;
    framecount = framecount + 1;
end
fprintf('Frame rate: %.1f fps\n', framecount/toc);
