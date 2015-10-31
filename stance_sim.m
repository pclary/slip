function [th, y, dx] = stance_sim(th0, y0, dx0, leq0, leqfun, params)
% Simulates a SLIP stance phase
% Inputs:
%   th0: touchdown angle
%   y0: previous apex height
%   dx0: forward velocity at previous apex
%   leqfun: equilibrium leg length as a function of (t, Y, leq0) where t is time
%     and Y = [l; dl; th; dth]
%   params: slip model parameters (see bislip_setup.m)
% Outputs:
%   th: takeoff angle
%   y: next apex height
%   dx: forward velocity at next apex

m = params(1);
k = params(4);
g = params(11);

% Y: [l; dl; th; dth]
odefun = @(t, Y)[Y(2); 
                 Y(1)*Y(4)^2 - g*cos(Y(3)) + k/m*(leqfun(t, Y, leq0) - Y(1));
                 Y(4);
                 (g*sin(Y(3)) - 2*Y(2)*Y(4))/Y(1)];
stopfun = @(t, Y) leqfun(t, Y, leq0) - Y(1);

% Get initial conditions for stance ODE
l0 = leq0;
hdiff = y0 - l0*cos(th0);
dy0 = -sqrt(2*g*hdiff);
dl0 = dy0*cos(th0) - dx0*sin(th0);
dth0 = -(dx0*cos(th0) + dy0*sin(th0))/l0;
Y0 = [l0; dl0; th0; dth0];

dtmax = 1e-2;
dtmin = 1e-3;

Y = odeshoot(odefun, Y0, dtmax, dtmin, stopfun, 1e2);

l = Y(1);
dl = Y(2);
th = Y(3);
dth = Y(4);
dx = -dl*sin(th) - dth*l*cos(th);
dy = dl*cos(th) - dth*l*sin(th);
y = l*cos(th) + dy^2/g/2;


function Yend = odeshoot(odefun, Y0, dtmax, dtmin, stopfun, tstop)
% stopfun(t, Y) should return >0 before stop condition, 0 at stop condition, 
% and <0 after stop condition
t = 0;
dt = dtmax;
Y = Y0;
while t < tstop
    Ynew = rk4step(Y, t, dt, odefun);
    sfval = stopfun(t, Ynew);
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
end
Yend = Y;


function Ynew = rk4step(Y, t, dt, odefun)
dY1 = odefun(t, Y);
Y1 = Y + dt/2*dY1;
dY2 = odefun(t + dt/2, Y1);
Y2 = Y + dt/2*dY2;
dY3 = odefun(t + dt/2, Y2);
Y3 = Y + dt*dY3;
dY4 = odefun(t + dt, Y3);
Ynew = Y + dt/6*(dY1 + 2*dY2 + 2*dY3 + dY4);
