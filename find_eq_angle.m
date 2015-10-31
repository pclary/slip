function th_eq = find_eq_angle(apex_height, forward_velocity, eq_length, params)

m = params(1);
k = params(4);
g = params(11);

% Y: [l; dl; th; dth]
odefun = @(t, Y)[Y(2); 
                 Y(1)*Y(4)^2 - g*cos(Y(3)) + k/m*(eq_length - Y(1));
                 Y(4);
                 (g*sin(Y(3)) - 2*Y(2)*Y(4))/Y(1)];
stopfun = @(Y) eq_length - Y(1);

    function Y0 = initial_conditions(th0)
        l0 = eq_length;
        hdiff = apex_height - l0*cos(th0);
        dx0 = forward_velocity;
        dy0 = -sqrt(2*g*hdiff);
        dl0 = dy0*cos(th0) - dx0*sin(th0);
        dth0 = -(dx0*cos(th0) + dy0*sin(th0))/l0;
        Y0 = [l0; dl0; th0; dth0];
    end

    function out = zerofun(th0)
        Y0 = initial_conditions(th0);
        dtmax = 1e-2;
        dtmin = 1e-3;
        Y = odeshoot(odefun, Y0, dtmax, dtmin, stopfun, 1e2);
        out = th0 + Y(3);
    end

th_eq = fzero(@zerofun, 0);
end


function Yend = odeshoot(odefun, Y0, dtmax, dtmin, stopfun, tstop)
% stopfun(Y) should return >0 before stop condition, 0 at stop condition, 
% and <0 after stop condition
t = 0;
dt = dtmax;
Y = Y0;
while t < tstop
    Ynew = rk4step(Y, t, dt, odefun);
    sfval = stopfun(Ynew);
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
end


function Ynew = rk4step(Y, t, dt, odefun)
dY1 = odefun(t, Y);
Y1 = Y + dt/2*dY1;
dY2 = odefun(t + dt/2, Y1);
Y2 = Y + dt/2*dY2;
dY3 = odefun(t + dt/2, Y2);
Y3 = Y + dt*dY3;
dY4 = odefun(t + dt, Y3);
Ynew = Y + dt/6*(dY1 + 2*dY2 + 2*dY3 + dY4);
end