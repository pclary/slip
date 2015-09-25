function dY = slip_stance(~, Y, m, k, l, g, toe)
% t: time (s)
% Y: state [x_COM (m); y_COM (m); dxdt_COM (m/s); dydt_COM (m/s)]
% m: mass (kg)
% k: total stiffness (leg + ground) (N/m)
% l: equilibrium leg length (m)
% g: gravity (m/s^2)
% toe: position of toe [x (m); y (m)]

% get leg vector and compressed length
leg = Y(1:2) - toe;
lc = norm(leg);

% get force on COM due to spring
F = -(leg/lc)*(k*(lc - l));

dY = [Y(3:4); F/m + [0; -g]];
