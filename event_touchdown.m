function [value, isterminal, direction] = event_touchdown(t, Y, l, controller, yground)
% t: time (s)
% Y: state [x_COM (m); y_COM (m); dxdt_COM (m/s); dydt_COM (m/s)]
% l: equilibrium leg length (m)
% controller: function of (t, Y) that gives the leg angle (+ccw from -y)
% yground: function of x that returns y-height of ground

% get toe position
th = controller(t, Y);
leg = l*[sin(th); -cos(th)];
toe = Y(1:2) + leg;

% check if toe is above ground
if Y(2) < yground(Y(1)) % also stop if COM is below ground
    value = 0;
    isterminal = 1;
    direction = 0;
else
    value = toe(2) - yground(toe(1));
    isterminal = 1;
    direction = -1;
end
