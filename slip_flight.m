function dY = slip_flight(~, Y, g)
% t: time (s)
% Y: state [x_COM (m); y_COM (m); dxdt_COM (m/s); dydt_COM (m/s)]
% g: gravity (m/s^2)

dY = [Y(3:4); 0; -g];
