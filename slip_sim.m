function [t, states, itdwn, itoff] = slip_sim(model_params, environment, controllers, states0, nsteps)
% Runs a SLIP simulation with the given model parameters, environment
% parameters, angle and length controllers, and initial conditions. The
% simulation stops when the model has made nsteps steps or when the center
% of mass falls into the ground.
% 
% Parameters:
%   model_params: structure with the following members
%     .mass: point mass of SLIP model (kg)
%     .stiffness: leg spring stiffness (N/m)
%     .damping: parallel damping with leg spring (N*s/m)
%     .gravity: positive downwards gravitational acceleration (m/s^2)
%   environment: structure with the following members
%     .ground_height: handle to function of x returning ground height
%   controllers: structure with the following members
%     .flight_angle: handle to function of (t, Y, states0) returning leg angle
%     .flight_length: handle to function of (t, Y, states0) returning leg length
%     .stance_length: handle to function of (t, Y, states0) returning leg length
%   states0: value of states vector (see below) at simulation starting
%     point, which should be takeoff of the previous step
%   nsteps: number of steps after which to terminate simulation
% 
% Output:
%   t: simulation time (s)
%   states: [com_x, com_y, com_xdot, com_ydot, angle, length_eq, length_compr, grf_x, grf_y]
%   itdwn: indices of touchdown events
%   itoff: indices of takeoff events

% Initialize simulation data 
states0 = states0(:)';
t = 0;
Y = states0(1:4);
angle = states0(5);
length_eq = states0(6);
length_comp = states0(7);
grf_x = states0(8);
grf_y = states0(9);
ttdwn = [];
ttoff = [];

% Time limits for individual steps
tspan = [0 1e1];

% Run simulation for up to n steps
for i = 1:nsteps
    
    % Simulate flight dynamics
    states0_f = [Y(end, :)'; angle(end); length_eq(end); length_comp(end); grf_x(end); grf_y(end)];
    touchdownfun = @(t, Y) event_touchdown(t, Y, controllers.flight_length, controllers.flight_angle, states0_f, environment.ground_height);
    fopts = odeset('Events', touchdownfun, 'Refine', 8, 'RelTol', 1e-5, 'AbsTol', 1e-10);
    flightfun = @(t, Y) slip_flight(t, Y, model_params.gravity);
    [t_f, Y_f] = ode45(flightfun, tspan, Y(end, :)', fopts);
    
    % Solve for angles during flight (keep toe out of ground regardless of
    % what controller wants)
    angle_f = zeros(length(t_f), 1);
    length_eq_f = zeros(length(t_f), 1);
    fsopts = optimoptions('fsolve', 'Display', 'off');
    for j = 1:length(angle_f)
        angle_f(j) = controllers.flight_angle(t_f(j), Y_f(j, :)', states0_f);
        length_eq_f(j) = controllers.flight_length(t_f(j), Y_f(j, :)', states0_f);
        toe = Y_f(j, 1:2)' + length_eq_f(j)*[sin(angle_f(j)); -cos(angle_f(j))];
        if environment.ground_height(toe(1)) > toe(2)
            fun = @(th) environment.ground_height(Y_f(j, 1) + length_eq_f(j)*sin(th)) - (Y_f(j, 2) - length_eq_f(j)*cos(th));
            angle_f(j) = fsolve(fun, angle_f(j), fsopts);
        end
    end
    
    % Concatenate results
    t = [t; t_f + t(end)];
    Y = [Y; Y_f];
    angle = [angle; angle_f];
    length_eq = [length_eq; length_eq_f];
    length_comp = [length_comp; length_eq_f]; % leg is at equilibrium length during flight
    grf_x = [grf_x; zeros(size(t_f))];
    grf_y = [grf_y; zeros(size(t_f))];
    
    % Stop if hopper crashed (COM below ground)
    if environment.ground_height(Y(end, 1)) >= Y(end, 2)
        break;
    end
    
    % Record touchdown time
    ttdwn = [ttdwn; t(end)];
    
    % Simulate stance dynamics
    states0_s = [Y(end, :)'; angle(end); length_eq(end); length_comp(end); grf_x(end); grf_y(end)];
    takeofffun =  @(t, Y) event_takeoff(t, Y, controllers.stance_length, states0_s, environment.ground_height);
    sopts = odeset('Events', takeofffun, 'Refine', 8, 'RelTol', 1e-5, 'AbsTol', 1e-10);
    keff = model_params.stiffness; % will eventually incorporate ground stiffness 
    beff = model_params.damping; % ^
    stancefun = @(t, Y) slip_stance(t, Y, model_params.mass, keff, beff, model_params.gravity, controllers.stance_length, states0_s);
    [t_s, Y_s] = ode45(stancefun, tspan, Y(end, :)', sopts);

    % Calculate angle and compressed length during stance
    leg = bsxfun(@minus, toe', Y_s(:, 1:2));
    angle_s = atan2(leg(:, 1), -leg(:, 2));
    length_comp_s = sqrt(leg(:, 1).^2 + leg(:, 2).^2);
    
    % Get controlled equilibrium length during stance and ground reaction forces
    length_eq_s = zeros(length(t_s), 1);
    grf_x_s = zeros(length(t_s), 1);
    grf_y_s = zeros(length(t_s), 1);
    for j = 1:length(t_s)
        length_eq_s(j) = controllers.stance_length(t_s(j), Y_s(j,:)', states0_s);
        dY = stancefun(t_s(j), Y_s(j, :)');
        grf_x_s(j) = model_params.mass*dY(3);
        grf_y_s(j) = model_params.mass*(dY(4) + model_params.gravity);
    end
    
    % Concatenate results
    t = [t; t_s + t(end)];
    Y = [Y; Y_s];
    angle = [angle; angle_s];
    length_eq = [length_eq; length_eq_s];
    length_comp = [length_comp; length_comp_s];
    grf_x = [grf_x; grf_x_s];
    grf_y = [grf_y; grf_y_s];
    
    % Stop if hopper crashed (COM below ground)
    if environment.ground_height(Y(end, 1)) >= Y(end, 2)
        break;
    end
    
    % Record takeoff time
    ttoff = [ttoff; t(end)];
end

% Remove duplicate points
[t, ia, ~] = unique(t);
Y = Y(ia, :);
angle = angle(ia, :);
length_eq = length_eq(ia, :);
length_comp = length_comp(ia, :);
grf_x = grf_x(ia, :);
grf_y = grf_y(ia, :);

% Arrange output
states = [Y, angle, length_eq, length_comp, grf_x, grf_y];
[~, itdwn, ~] = intersect(t, ttdwn);
[~, itoff, ~] = intersect(t, ttoff);
