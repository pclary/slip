function [value, isterminal, direction] = event_touchdown(t, Y, flight_length, flight_angle, states0, ground_height)
% t: time (s)
% Y: state [x_COM (m); y_COM (m); dxdt_COM (m/s); dydt_COM (m/s)]
% flight_length: leg length controller, function of (t, Y, states0)
% flight_angle: leg angle controller, function of (t, Y, states0)
% states0: values of [com_x, com_y, com_xdot, com_ydot, angle, length_eq, 
%   length_compr, grf_x, grf_y] at takeoff
% ground_height: function of x that returns y-height of ground

% get toe position
th = flight_angle(t, Y, states0);
l = flight_length(t, Y, states0);
leg = l*[sin(th); -cos(th)];
toe = Y(1:2) + leg;

% check if toe is above ground
if Y(2) < ground_height(Y(1)) % also stop if COM is below ground
    value = 0;
    isterminal = 1;
    direction = 0;
else
    value = toe(2) - ground_height(toe(1));
    isterminal = 1;
    direction = -1;
end
