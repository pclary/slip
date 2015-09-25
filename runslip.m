%% Setup
% SLIP parameters
m = 1;
k = 100;
l = 1;
g = 1;

% Ground height function
yground = @(x) 0*ones(size(x));

% Swing leg controller
controller = @(t, Y) pi/4;

% Initial conditions
Y0 = [0; 1.1; 0.1; 0];

fopts = odeset('Events', @(t, Y) event_impact(t, Y, l, controller, yground));
nsteps = 2;
timeout = 1e3;

tt = 0;
YY = Y0';
th = controller(0, Y0);
Toe = Y0(1:2)' + l*[sin(th), -cos(th)];


%% Run simulation
for i = 1:nsteps
    % Flight phase
    [t, Y] = ode45(@(t, Y) slip_flight(t, Y, g), [0 timeout], Y0, fopts);
    Y0 = Y(end, :)';
    tt = [tt; t + tt(end)];
    YY = [YY; Y];
    th = zeros(length(t), 1);
    for j = 1:length(th)
        th(j) = controller(t(j), Y(j, :)');
    end
    Toe = [Toe; bsxfun(@plus, Y(:, 1:2), l*[sin(th), -cos(th)])];
    
    % Stop if hopper crashed (COM below ground)
    if yground(Y0(1)) >= Y0(2)
        break;
    end
    
    % Stance phase
    th = controller(t(end), Y0);
    toe = Y0(1:2) + l*[sin(th); -cos(th)];
    sopts = odeset('Events', @(t, Y) event_takeoff(t, Y, l, toe));
    [t, Y] = ode45(@(t, Y) slip_stance(t, Y, m, k, l, g, toe), [0 timeout], Y0, sopts);
    Y0 = Y(end, :)';
    tt = [tt; t + tt(end)];
    YY = [YY; Y];
    Toe = [Toe; repmat(toe', length(t), 1)];
    
    % Stop if hopper crashed (COM below ground)
    if yground(Y0(1)) >= Y0(2)
        break;
    end
end

%% Display
sg = SlipGraphics();

for i = 1:length(t);
    sg.setState(YY(i, 1:2), Toe(i, :));
    sg.setGround(yground, 1e3);
end
