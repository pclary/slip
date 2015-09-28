function [value, isterminal, direction] = event_takeoff(t, Y, lsctrl, toe, yground)
% t: time (s)
% Y: state [x_COM (m); y_COM (m); dxdt_COM (m/s); dydt_COM (m/s)]
% l: equilibrium leg length (m)
% controller: function of (t, Y) that gives the leg angle (+ccw from -y)
% yground: function of x that returns y-height of ground

% get leg vector and compressed length
l = lsctrl(t, Y);
leg = Y(1:2) - toe;
lc = norm(leg);

% check if leg is no longer compressed
if Y(2) < yground(Y(1)) % also stop if COM is below ground
    value = 0;
    isterminal = 1;
    direction = 0;
else
    value = lc - l;
    isterminal = 1;
    direction = 1;
end
