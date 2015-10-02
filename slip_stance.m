function dY = slip_stance(t, Y, m, k, b, g, stance_length, states0)
% t: time (s)
% Y: state [com_x (m); com_y (m); com_xdot (m/s); com_ydot (m/s)]
% m: mass (kg)
% k: total stiffness (leg + ground) (N/m)
% b: total damping (N*s/m)
% g: gravity (m/s^2)
% stance_length: leg length controller, function of (t, Y, states0)
% states0: values of [com_x, com_y, com_xdot, com_ydot, angle, length_eq, 
%   length_compr, grf_x, grf_y] at touchdown

% get leg vector, compressed length, and rate of compression
l = stance_length(t, Y, states0);
toe = states0(1:2) + states0(7)*[sin(states0(5)); -cos(states0(5))];
leg = Y(1:2) - toe;
lc = norm(leg);
dlc = dot(Y(3:4), leg)/l;

% get force on COM due to spring
F = -(leg/lc)*(k*(lc - l) + b*dlc);

dY = [Y(3:4); F/m + [0; -g]];
