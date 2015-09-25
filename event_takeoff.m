function [value, isterminal, direction] = event_takeoff(~, Y, l, toe)
% t: time (s)
% Y: state [x_COM (m); y_COM (m); dxdt_COM (m/s); dydt_COM (m/s)]
% l: equilibrium leg length (m)
% controller: function of (t, Y) that gives the leg angle (+ccw from -y)
% yground: function of x that returns y-height of ground

% get leg vector and compressed length
leg = Y(1:2) - toe;
lc = norm(leg);

% check if toe is above ground
value = lc - l;
isterminal = 1;
direction = 1;
