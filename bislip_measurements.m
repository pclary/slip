function Y = bislip_measurements(X)
% Y: [body_angle; leg_a_angle; leg_a_length_eq; leg_a_length; leg_b_angle; leg_b_length_eq; leg_b_length]

body_angle = X(5);
leg_a_vec = X(7:8) - X(1:2);
leg_a_angle = atan2(leg_a_vec(1), -leg_a_vec(2)) - body_angle;
leg_a_length_eq = X(11);
leg_a_length = sqrt(leg_a_vec(1)^2 + leg_a_vec(2)^2);
leg_b_vec = X(12:13) - X(1:2);
leg_b_angle = atan2(leg_b_vec(1), -leg_b_vec(2)) - body_angle;
leg_b_length_eq = X(16);
leg_b_length = sqrt(leg_b_vec(1)^2 + leg_b_vec(2)^2);

Y = [body_angle; 
     leg_a_angle; leg_a_length_eq; leg_a_length; 
     leg_b_angle; leg_b_length_eq; leg_b_length];
