function [dX, ground_force_a, ground_force_b, u_limit] = bislip_dynamics(X, u, params, ground_data, body_ext_force)
% X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
%     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
%     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
% u: [length_motor_a_force; angle_motor_a_torque;
%     length_motor_b_force; angle_motor_b_torque]
% params: [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping; 
%          length_motor_inertia; length_motor_damping; angle_motor_inertia; 
%          angle_motor_damping; angle_motor_ratio; gravity]
% ground_data: [ground_x, ground_y, ground_stiffness, 
%               ground_damping, ground_friction]

% Kinematics
% body: [x; xdot; y; ydot; th; thdot]
% leg: [leq; leqdot; l; ldot; th; thdot; x; xdot; y; ydot; xdir; ydir]
% Leg angles are relative to body angle
% Leg x, y, xdot, ydot are absolute
body = X(1:6);
leg_a = leg_kinematics(X(7:12), body);
leg_b = leg_kinematics(X(13:18), body);

% Calculate ground force for each leg
ground_force_a = ground_contact_model(leg_a([7 9]), leg_a([8 10]), ground_data);
ground_force_b = ground_contact_model(leg_b([7 9]), leg_b([8 10]), ground_data);

% Calculate ground force on body
body_ground_force = ground_contact_model(body([1 3]) + [0; -0.1], body([2 4]), ground_data);

% Hard stop forces
u_limit = limit_forces(X);

% Use big ugly jacobian to get most of the derivatives
u_full = [u + u_limit; body_ground_force + body_ext_force; 0; ground_force_a; ground_force_b];
dX = bislip_eom(X, u_full, params);


function u_limit = limit_forces(X)
% Calculate hard stop forces
kp_leq = 4e5;
kd_leq = 4e3;
kp_th = 1e4;
kd_th = 1e3;
dfade_leq = 0.001;
dfade_th = 0.001;

leq_max = 1.5;
leq_min = 0.5;
th_max = pi/2;
th_min = -pi/2;

leq_a_over = min(X(7) - leq_min, max(X(7) - leq_max, 0));
leq_b_over = min(X(13) - leq_min, max(X(13) - leq_max, 0));
th_a_over = min(X(11) - th_min, max(X(11) - th_max, 0));
th_b_over = min(X(17) - th_min, max(X(17) - th_max, 0));

leq_a_dfade = fade_derivative(X(7), leq_min, leq_max, dfade_leq);
leq_b_dfade = fade_derivative(X(13), leq_min, leq_max, dfade_leq);
th_a_dfade = fade_derivative(X(11), th_min, th_max, dfade_th);
th_b_dfade = fade_derivative(X(17), th_min, th_max, dfade_th);

leq_a_limit_force = -kp_leq*leq_a_over - kd_leq*X(8)*leq_a_dfade;
leq_b_limit_force = -kp_leq*leq_b_over - kd_leq*X(14)*leq_b_dfade;
th_a_limit_torque = -kp_th*th_a_over - kd_th*X(12)*th_a_dfade;
th_b_limit_torque = -kp_th*th_b_over - kd_th*X(18)*th_b_dfade;

limit_force_max = [1e6; 1e4];

u_limit = [leq_a_limit_force; th_a_limit_torque; leq_b_limit_force; th_b_limit_torque];
u_limit = min(max(u_limit, -[limit_force_max; limit_force_max]), [limit_force_max; limit_force_max]);


function dfade = fade_derivative(x, x_min, x_max, fade_width)
dfade = abs(min(max((x - x_min - fade_width)/fade_width, -1), ...
                max(min((x - x_max + fade_width)/fade_width, 1), 0)));


function leg = leg_kinematics(X_leg, body)
% Calculate lengths, derivatives, etc
leg = zeros(12, 1);
leg(1:6) = X_leg(1:6); % th, leg, l, and derivatives of each
leg(11:12) = [sin(leg(5) + body(5)); -cos(leg(5) + body(5))]; % leg direction unit vector, absolute
leg([7 9]) = body([1 3]) + leg(3)*leg(11:12); % foot x and y
leg([8 10]) = body([2 4]) + leg(4)*leg(11:12) ...
    + leg(3)*(body(6) + leg(6))*[-leg(12); leg(11)]; % foot xdot and ydot


function ground_force = ground_contact_model(pos, vel, ground_data)
% Ground contact force model
% Takes position and velocity of point that forces act on, a reference
% position used to find the correct ground intersection location, external
% forces on the point, and the ground data structure
% Returns the ground forces and a structure containing intermediate values

[inground, ground_normal, xc, yc, ic, pc] = test_ground(pos, ground_data);
ground_tangent = [ground_normal(2); -ground_normal(1)];

if inground
    % Find depth into ground, speed, and ground properties at contact
    depth = norm([xc - pos(1); yc - pos(2)]);
    ddepth = -vel(1)*ground_normal(1) - vel(2)*ground_normal(2);
    ground_stiffness = interpolate(ground_data(:, 3), ic, pc);
    ground_damping = interpolate(ground_data(:, 4), ic, pc);
    ground_friction = interpolate(ground_data(:, 5), ic, pc);
    
    % Ramp up damping with depth
    damping_threshold = 1e-5;
    ground_damping = ground_damping*depth/(depth + damping_threshold);
else
    % No ground contact
    depth = 0;
    ddepth = 0;
    ground_stiffness = 0;
    ground_damping = 0;
    ground_friction = 0;
end

% Ground reaction force from spring-damper system
% Acts nomral to the ground
spring_force = ground_normal*max(depth*ground_stiffness + ddepth*ground_damping, 0);

% Friction magnitude is proportional to component of ground spring force
% normal to ground
friction_mag = ground_friction*(spring_force(1)*ground_normal(1) + spring_force(2)*ground_normal(2));

% Calculate friction, replacing the jump discontinuity due to the sign
% function with a continuous ramp
ground_slip = vel(1)*ground_tangent(1) + vel(2)*ground_tangent(2);
slip_ramp_width = friction_mag*1e-5;
p = min(max(abs(ground_slip)/slip_ramp_width, 0), 1);
friction_force = -sign(ground_slip)*p*friction_mag*ground_tangent;

% Total ground force
ground_force = spring_force + friction_force;


function out = interpolate(v, i, p)
% Interpolation function for ground properties
out = v(i) + p*(v(i+1) - v(i));
