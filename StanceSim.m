classdef StanceSim < matlab.System
    
     properties
        params = zeros(11, 1);
        % params: [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping; 
        %          length_motor_inertia; length_motor_damping; angle_motor_inertia;
        %          angle_motor_damping; angle_motor_ratio; gravity]
        mode = int32(0);
    end
    
    properties (DiscreteState)
    end
    
    methods (Access = protected)
        function setupImpl(obj)
        end
        
        function out = stepImpl(obj)
            
            
        end
        
        function resetImpl(obj)
        end
    end
    
    methods (Access = private)
        function dY = stance_dynamics(t, Y)
            m = obj.params(1);
            k = obj.params(4);
            g = obj.params(11);
            dY = [Y(2);
                  Y(1)*Y(4)^2 - g*cos(Y(3)) + k/m*(leqfun(t, Y, leq0) - Y(1));
                  Y(4);
                  (g*sin(Y(3)) - 2*Y(2)*Y(4))/Y(1)];
        end
    end
end


function [x2, f2] = newton(fun, x0, f0, dx)
x1 = x0 + dx;
f1 = fun(x1);
df = f1 - f0;
x2 = x0 - f0/(df/dx);
f2 = f(x2);


function Yend = odeshoot(odefun, Y0, dtmax, dtmin, stopfun, tstop)
% Integrates odefun until stopfun triggers or tstop is reached
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
end


function Ynew = rk4step(Y, t, dt, odefun)
% Single runge-kutta step
dY1 = odefun(t, Y);
Y1 = Y + dt/2*dY1;
dY2 = odefun(t + dt/2, Y1);
Y2 = Y + dt/2*dY2;
dY3 = odefun(t + dt/2, Y2);
Y3 = Y + dt*dY3;
dY4 = odefun(t + dt, Y3);
Ynew = Y + dt/6*(dY1 + 2*dY2 + 2*dY3 + dY4);
end
