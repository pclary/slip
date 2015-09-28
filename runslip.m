%% Setup
% SLIP parameters
m = 10;
k = 1000;
b = 0;
l = 1;
g = 9.81;

% Ground height function
yground = @(x) 0*x;

% Swing leg controller
controller = @(t, Y) 0.07705;

% Initial conditions
Y0 = [0; 1.5; 0.5; 0];

fopts = odeset('Events', @(t, Y) event_touchdown(t, Y, l, controller, yground));
nsteps = 30;
timeout = 1e1;

t = 0;
Y = Y0';
th = controller(0, Y0);
Toe = Y0(1:2)' + l*[sin(th), -cos(th)];
Ystep = [];

%% Run simulation
for i = 1:nsteps
    % Flight phase
    [tp, Yp] = ode45(@(t, Y) slip_flight(t, Y, g), [0 timeout], Y0, fopts);
    Y0 = Yp(end, :)';
    t = [t; tp + t(end)];
    Y = [Y; Yp];
    th = zeros(length(tp), 1);
    for j = 1:length(th)
        th(j) = controller(tp(j), Yp(j, :)');
        if yground(Yp(j, 1) + l*sin(th(j))) > Yp(j, 2) - l*cos(th(j))
            fun = @(th) yground(Yp(j, 1) + l*sin(th)) - (Yp(j, 2) - l*cos(th));
            fsopts = optimoptions('fsolve', 'Display', 'off');
            th(j) = fsolve(fun, th(j), fsopts);
        end
    end
    Toe = [Toe; bsxfun(@plus, Yp(:, 1:2), l*[sin(th), -cos(th)])];
    toe = Toe(end, :)';
    
    % Stop if hopper crashed (COM below ground)
    if yground(Y0(1)) >= Y0(2)
        break;
    end
    
    % Stance phase
    Ystep = [Ystep; [toe', t(end)]];
    sopts = odeset('Events', @(t, Y) event_takeoff(t, Y, l, toe, yground));
    keff = k;
    beff = b;
    [tp, Yp] = ode45(@(t, Y) slip_stance(t, Y, m, keff, beff, l, g, toe), [0 timeout], Y0, sopts);
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
if exist('sg', 'var') && isa(sg, 'SlipGraphics') && sg.isAlive()
    sg.clearTrace();
else
    sg = SlipGraphics();
end

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

%% Analysis
% Energy
spring = sqrt((Y(:,1) - Toe(:,1)).^2 + (Y(:,2) - Toe(:,2)).^2);
kg = kground(Toe(:,1));
keff = (k.*kg)./(k + kg);

GPE = m*g*Y(:,2);
KE = 1/2*m*(Y(:,3).^2 + Y(:,4).^2);
SPE = 1/2*keff.*(l - spring).^2;

if exist('enax', 'var') && isa(enax, 'matlab.graphics.axis.Axes') && enax.isvalid()
    cla(enax);
else
    energyfig = figure;
    enax = axes('Parent', energyfig);
end
plot(enax, t, GPE, t, KE, t, SPE, t, GPE+KE+SPE);
title(enax, 'Enegry');
xlabel(enax, 'Time (s)');
ylabel(enax, 'Energy (J)');
legend(enax, 'GPE', 'KE', 'SPE', 'Total');

% Ground reaction force
t2 = t(1:end-1) + diff(t)/2;
ax = diff(Y(:,3))./diff(t);
ay = diff(Y(:,4))./diff(t);
grfx = ax*m;
grfy = (ay + g)*m;

if exist('grfax', 'var') && isa(grfax, 'matlab.graphics.axis.Axes') && grfax.isvalid()
    cla(grfax);
else
    grffig = figure;
    grfax = axes('Parent', grffig);
end
plot(grfax, t2, grfx, t2, grfy);
title(grfax, 'Ground Reaction force');
xlabel(grfax, 'Time (s)');
ylabel(grfax, 'Force (N)');
legend(grfax, 'GRF_x', 'GRF_y');




