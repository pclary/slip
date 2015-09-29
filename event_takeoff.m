function [value, isterminal, direction] = event_takeoff(t, Y, stance_length, states0, ground_height)
% t: time (s)
% Y: state [x_COM (m); y_COM (m); dxdt_COM (m/s); dydt_COM (m/s)]
% stance_length: leg length controller, function of (t, Y, states0)
% states0: values of [com_x, com_y, com_xdot, com_ydot, angle, length_eq, 
%   length_compr, grf_x, grf_y] at touchdown
% ground_height: function of x that returns y-height of ground

% get leg vector and compressed length
l = stance_length(t, Y, states0);
toe = states0(1:2) + states0(7)*[sin(states0(5)); -cos(states0(5))];
leg = Y(1:2) - toe;
lc = norm(leg);

% check if leg is no longer compressed
if Y(2) < ground_height(Y(1)) % also stop if COM is below ground
    value = 0;
    isterminal = 1;
    direction = 0;
else
    value = lc - l;
    isterminal = 1;
    direction = 1;
end
