function dY = slip_flight(~, Y, g)
% t: time (s)
% Y: state [com_x (m); com_y (m); com_xdot (m/s); com_ydot (m/s)]
% g: gravity (m/s^2)

dY = [Y(3:4); 0; -g];
