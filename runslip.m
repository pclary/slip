%% Setup
% SLIP parameters
model_params.mass = 10;
model_params.stiffness = 1000;
model_params.damping = 0;
model_params.gravity = 9.81;

% Ground height function
environment.ground_height = @(x) 0*x;

% Flight leg angle and flight/stance leg length controllers
controllers.flight_angle = @(t, Y) 0.075;
controllers.flight_length = @(t, Y) 1;
controllers.stance_length = @(t, Y) 1;

% Initial conditions
Y0 = [0; 1.5; 0.5; 0];

nsteps = 10;

[t, states, itdwn, itoff] = slip_sim(model_params, environment, controllers, Y0, nsteps);

%% Display
if exist('sg', 'var') && isa(sg, 'SlipGraphics') && sg.isAlive()
    sg.clearTrace();
else
    sg = SlipGraphics();
end

%Resampling doesn't currently keep the toe above the ground correctly
% % Resample trajectories with a fixed timestep
% ts = 1e-2;
% tr = 0:ts:max(t);
% statesr = interp1(t, states, tr);

for i = 1:length(t);
    sg.setState(states(i,1:2), states(i,5), states(i,7));
    sg.setGround(environment.ground_height, 1e3);
    isteps = itdwn(t(itdwn) <= t(i));
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
title(enax, 'Enegry');
xlabel(enax, 'Time (s)');
ylabel(enax, 'Energy (J)');
legend(enax, 'GPE', 'KE', 'SPE', 'Total');

% Ground reaction force
t2 = t(1:end-1) + diff(t)/2;
ax = diff(states(:,3))./diff(t);
ay = diff(states(:,4))./diff(t);
grfx = ax*model_params.mass;
grfy = (ay + model_params.gravity)*model_params.mass;

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
