function [energy, energies] = gait_energy(X, targets, kp, params)
% Compute gait energy and gait energy components

m = params(1);
k = params(4);
g = params(11);
err = X([7 11 5 13 17 5]) - targets;

% Physical energy
spring_a_energy = 1/2*k*(X(7) - X(9))^2;
spring_b_energy = 1/2*k*(X(13) - X(15))^2;
kinetic_energy = 1/2*m*(X(2)^2 + X(4)^2);
gravitational_energy = m*g*(X(3) - 1);

% Energy in controllers
controller_energies = 1/2*kp.*err.^2;
leq_controller_energy = sum(controller_energies([1 4]));
angle_controller_energy = sum(controller_energies([2 3 5 6]));

% Total energy
energies = [spring_a_energy; spring_b_energy; kinetic_energy; gravitational_energy; ...
    leq_controller_energy; angle_controller_energy; 0];
energy = sum(energies(:));
energies(end) = energy;
