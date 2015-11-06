function [th, y, dx] = stance_sim(th0, y0, dx0, leq0, params)
% Simulates a SLIP stance phase
% Inputs:
%   th0: touchdown angle
%   y0: previous apex height
%   dx0: forward velocity at previous apex
%   leq0: equilibrium length (and actual length) at touchdown
%   leqfun: equilibrium leg length as a function of (t, Y, leq0) where t is time
%     and Y = [l; dl; th; dth]
%   params: slip model parameters (see bislip_setup.m)
% Outputs:
%   th: takeoff angle
%   y: next apex height
%   dx: forward velocity at next apex

g = params(11);

% Get initial conditions for stance ODE
l0 = leq0;
hdiff = y0 - l0*cos(th0);
if hdiff < 0
    % Abort if initial conditions are already past touchdown
    th = NaN;
    y = NaN;
    dx = NaN;
    return;
end
dy0 = -sqrt(2*g*hdiff);
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
while t < tstop
    % Runge-kutta integration step
    dY1 = stance_dynamics(t, Y, leq0, params);
    Y1 = Y + dt/2*dY1;
    dY2 = stance_dynamics(t + dt/2, Y1, leq0, params);
    Y2 = Y + dt/2*dY2;
    dY3 = stance_dynamics(t + dt/2, Y2, leq0, params);
    Y3 = Y + dt*dY3;
    dY4 = stance_dynamics(t + dt, Y3, leq0, params);
    Ynew = Y + dt/6*(dY1 + 2*dY2 + 2*dY3 + dY4);
    
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
y = l*cos(th) + dy^2/g/2;


function dY = stance_dynamics(t, Y, leq0, params)
% Y: [l; dl; th; dth]
m = params(1);
k = params(4);
b = params(5);
g = params(11);

dY = [Y(2);
      Y(1)*Y(4)^2 - g*cos(Y(3)) + k/m*(leq0 - Y(1)) + b/m*(0 - Y(2));
      Y(4);
      (g*sin(Y(3)) - 2*Y(2)*Y(4))/Y(1)];
