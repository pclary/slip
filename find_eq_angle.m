function th_eq = find_eq_angle(apex_height, forward_velocity, eq_length, params)

m = params(1);
k = params(4);
g = params(11);

% Y: [l; dl; th; dth]
odefun = @(t, Y)[Y(2); 
                 Y(1)*Y(4)^2 - g*cos(Y(3)) + k/m*(eq_length - Y(1));
                 Y(4);
                 (g*sin(Y(3)) - 2*Y(2)*Y(4))/Y(1)];
eventfun = @(t, Y) takeoff_event(t, Y, eq_length);
odeopts = odeset('Events', eventfun);

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
        [~, Y] = ode45(odefun, [0 1e2], Y0, odeopts);
        out = th0 + Y(end, 3);
    end

th_eq = fzero(@zerofun, 0);
end


function [value, isterminal, direction] = takeoff_event(~, Y, eq_length)
value = [eq_length - Y(1); abs(Y(3)) - pi/2];
isterminal = [1; 1];
direction = [-1; 1];
end
