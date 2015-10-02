%% Setup
% SLIP parameters
m = 10;
k = 1000;
b = 10;
g = 9.81;
model_params.mass = m;
model_params.stiffness = k;
model_params.damping = b;
model_params.gravity = g;

% Ground height function
environment.ground_height = @(x) 1*sin(x/2);

% Flight leg angle and flight/stance leg length controllers
toe_height = @(states) states(2) - states(6)*cos(states(5));
controllers.flight_angle = @(t, Y, states0) (Y(3)-0.2)*0.2;
controllers.flight_length = @(t, Y, states0) 1;
controllers.stance_length = @(t, Y, states0) 1 - 1e-2*t*(energy(states0, model_params) - m*g*(2 + toe_height(states0)));

% Initial conditions
states0 = [0; 1.5; 0.5; 0; -0.077; 1; 1; 0; 0];

nsteps = 30;

[t, states, itdwn, itoff] = slip_sim(model_params, environment, controllers, states0, nsteps);

%% Display
if exist('sg', 'var') && isa(sg, 'SlipGraphics') && sg.isAlive()
    sg.clearTrace();
else
    sg = SlipGraphics();
end

% Resample trajectories with a fixed timestep
toe = states(:,1:2) + bsxfun(@times, states(:,7), [sin(states(:,5)), -cos(states(:,5))]);
ts = 1e-2;
tr = 0:ts:max(t);
statesr = interp1(t, states, tr);
toer = interp1(t, toe, tr);

for i = 1:length(tr);
    sg.setState(statesr(i,1:2), toer(i,:));
    sg.setGround(environment.ground_height, 1e3);
    isteps = itdwn(t(itdwn) <= tr(i));
    steppts = states(isteps,1:2) + ...
        bsxfun(@times, states(isteps,7), [sin(states(isteps,5)), -cos(states(isteps,5))]);
    sg.setSteps(steppts(:,1), steppts(:,2));
    drawnow;
end

%% Analysis
% Energy
keff = model_params.stiffness;
GPE = model_params.mass*model_params.gravity*states(:,2);
KE = 1/2*model_params.mass*(states(:,3).^2 + states(:,4).^2);
SPE = 1/2*keff.*(states(:,6) - states(:,7)).^2;

if exist('enax', 'var') && isa(enax, 'matlab.graphics.axis.Axes') && enax.isvalid()
    cla(enax);
else
    energyfig = figure;
    enax = axes('Parent', energyfig);
end
plot(enax, t, GPE, t, KE, t, SPE, t, GPE+KE+SPE);
title(enax, 'Energy');
xlabel(enax, 'Time (s)');
ylabel(enax, 'Energy (J)');
legend(enax, 'GPE', 'KE', 'SPE', 'Total');

% Ground reaction force

if exist('grfax', 'var') && isa(grfax, 'matlab.graphics.axis.Axes') && grfax.isvalid()
    cla(grfax);
else
    grffig = figure;
    grfax = axes('Parent', grffig);
end
plot(grfax, t, states(:,8), t, states(:,9));
title(grfax, 'Ground Reaction force');
xlabel(grfax, 'Time (s)');
ylabel(grfax, 'Force (N)');
legend(grfax, 'GRF_x', 'GRF_y');
