%% Setup
% SLIP parameters
m = 1;
k = 10;
l = 1;
g = 1;

% Ground height and stiffness functions
yground = @(x) 0*ones(size(x));
kground = @(x) 1e6*ones(size(x));

% Swing leg controller
controller = @(t, Y) 0.3;

% Initial conditions
Y0 = [0; 1.1; 0.5; 0];

fopts = odeset('Events', @(t, Y) event_impact(t, Y, l, controller, yground));
nsteps = 30;
timeout = 1e1;

t = 0;
Y = Y0';
th = controller(0, Y0);
Toe = Y0(1:2)' + l*[sin(th), -cos(th)];
Ystep = [];

%% Run simulation
for i = 1:nsteps
    % Make sure initial controller position has toe above ground
    th = controller(t(end), Y0);
    toe = Y0(1:2) + l*[sin(th); -cos(th)];
    if yground(toe(1)) > toe(2)
        % Skip flight phase; go to stance with angle at which toe first
        % hits the ground
        fun = @(th) yground(Y0(1) + l*sin(th)) - (Y0(2) - l*cos(th));
        leg = Toe(end, :)' - Y0(1:2);
        th0 = atan2(leg(1), -leg(2));
        fsopts = optimoptions('fsolve', 'Display', 'off');
        th = fsolve(fun, th0, fsopts);
        toe = Y0(1:2) + l*[sin(th); -cos(th)];
        toe(2) = yground(toe(1));
    else
        % Flight phase
        Toe(end, :) = [nan nan];
        [tp, Yp] = ode45(@(t, Y) slip_flight(t, Y, g), [0 timeout], Y0, fopts);
        Y0 = Yp(end, :)';
        t = [t; tp + t(end)];
        Y = [Y; Yp];
        th = zeros(length(tp), 1);
        for j = 1:length(th)
            th(j) = controller(tp(j), Yp(j, :)');
        end
        Toe = [Toe; bsxfun(@plus, Yp(:, 1:2), l*[sin(th), -cos(th)])];
        toe = Toe(end, :)';
    end
    
    % Stop if hopper crashed (COM below ground)
    if yground(Y0(1)) >= Y0(2)
        break;
    end
    
    % Stance phase
    Ystep = [Ystep; [toe', t(end)]];
    sopts = odeset('Events', @(t, Y) event_takeoff(t, Y, l, toe, yground));
    ktot = (k*kground(toe(1)))/(k + kground(toe(1)));
    [tp, Yp] = ode45(@(t, Y) slip_stance(t, Y, m, ktot, l, g, toe), [0 timeout], Y0, sopts);
    Y0 = Yp(end, :)';
    t = [t; tp + t(end)];
    Y = [Y; Yp];
    Toe = [Toe; repmat(toe', length(tp), 1)];
    
    % Stop if hopper crashed (COM below ground)
    if yground(Y0(1)) >= Y0(2)
        break;
    end
end

% Remove duplicate points
[t, ia, ~] = unique(t);
Y = Y(ia, :);
Toe = Toe(ia, :);

%% Display
sg = SlipGraphics();

% Resample trajectories with a fixed timestep
ts = 1e-2;
tr = 0:ts:max(t);
Yr = interp1(t, Y, tr);
Toer = interp1(t, Toe, tr);

for i = 1:length(tr);
    sg.setState(Yr(i, 1:2), Toer(i, :));
    sg.setGround(yground, 1e3);
    istep = find(Ystep(:, 3) <= tr(i), 1, 'last');
    sg.setSteps(Ystep(1:istep, 1), Ystep(1:istep, 2));
    drawnow;
end
