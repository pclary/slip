function Y = bislip_measurements(X)
% X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
%     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
%     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
% Y: [body_angle;
%     leg_a_leq; leg_a_l; leg_a_th; 
%     leg_b_leq; leg_b_l; leg_b_th]

Y = X([5 7 9 11 13 15 17]);
