function [th, dx, dy] = stance_sim(th0, dx0, dy0, leq0, leq_ext, params)
% Simulates a SLIP stance phase
% Inputs:
%   th0: touchdown angle
%   dx0: forward velocity at touchdown
%   dy0: vertical velocity at touchdown
%   leq0: equilibrium length (and actual length) at touchdown
%   leq_ext: amount the leg is to be extended during stance
%   params: slip model parameters (see bislip_setup.m)
% Outputs:
%   th: takeoff angle
%   dx: forward velocity at takeoff
%   dy: vertical velocity at takeoff

% Get initial conditions for stance ODE
l0 = leq0;
dl0 = dy0*cos(th0) - dx0*sin(th0);
dth0 = -(dx0*cos(th0) + dy0*sin(th0))/l0;
Y0 = [l0; dl0; th0; dth0];

dtmax = 1e-2;
dtmin = 1e-3;
tstop = 1e1;

% Integrate stance dynamics until takeoff or timeout
t = 0;
dt = dtmax;
Y = Y0;
t_ext = inf;
while t < tstop
    % Runge-kutta integration step
    dY1 = stance_dynamics(t, Y, leq0, leq_ext, t_ext, params);
    Y1 = Y + dt/2*dY1;
    dY2 = stance_dynamics(t + dt/2, Y1, leq0, leq_ext, t_ext, params);
    Y2 = Y + dt/2*dY2;
    dY3 = stance_dynamics(t + dt/2, Y2, leq0, leq_ext, t_ext, params);
    Y3 = Y + dt*dY3;
    dY4 = stance_dynamics(t + dt, Y3, leq0, leq_ext, t_ext, params);
    Ynew = Y + dt/6*(dY1 + 2*dY2 + 2*dY3 + dY4);
    
    % Find leg extension starting time
    if ~isfinite(t_ext) && Ynew(2) > 0 && Y(2) <= 0
        t_ext = t;
    end
    
    % Stopping logic
    sfval = leq0 - Ynew(1);
    if sfval <= 0
        if dt/2 >= dtmin
            dt = dt/2;
            continue;
        else
            Y = Ynew;
            break;
        end
    else
        Y = Ynew;
        t = t + dt;
    end
    if Y(1) <= 0
        % Stop if spring fully compresses
        Y = Y*NaN;
        break;
    end
end
if t >= tstop
    % Mark as invalid
    Y = Y*NaN;
end

% Compute outputs
l = Y(1);
dl = Y(2);
th = Y(3);
dth = Y(4);
dx = -dl*sin(th) - dth*l*cos(th);
dy = dl*cos(th) - dth*l*sin(th);


function dY = stance_dynamics(t, Y, leq0, leq_ext, t_ext, params)
% Y: [l; dl; th; dth]
m = params(1);
k = params(4);
b = params(5);
g = params(11);

if t > t_ext
    extension_time = 0.2;
    extension_rate = leq_ext/extension_time;
    leq = leq0 + min((t - t_ext)*extension_rate, leq_ext);
    dleq = extension_rate;
else
    leq = leq0;
    dleq = 0;
end

dY = [Y(2);
      Y(1)*Y(4)^2 - g*cos(Y(3)) + k/m*(leq - Y(1)) + b/m*(dleq - Y(2));
      Y(4);
      (g*sin(Y(3)) - 2*Y(2)*Y(4))/Y(1)];
