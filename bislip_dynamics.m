function [dX, body, leg_a, leg_b] = bislip_dynamics(X, u, params, ground_data)
% X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
%     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
%     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
% u: [length_motor_a_torque; angle_motor_a_torque;
%     length_motor_b_torque; angle_motor_b_torque]
% params: [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping; 
%         length_motor_inertia; length_motor_ratio; length_motor_damping; 
%         angle_motor_inertia;  angle_motor_ratio;  angle_motor_damping; 
%         gravity]
% ground_data: [ground_x, ground_y, ground_stiffness, 
%               ground_damping, ground_friction]

% Kinematics
% body: [x; xdot; y; ydot; th; thdot]
% leg: [leq; leqdot; l; ldot; th; thdot; x; xdot; y; ydot; xdir; ydir]
body = X(1:6);
leg_a = leg_kinematics(X(7:12), body);
leg_b = leg_kinematics(X(13:18), body);

% Calculate dynamics for each leg
[leg_a_leqddot, leg_a_lddot, leg_a_thddotabs, reaction_force_a, reaction_torque_a] ...
    = leg_dynamics(leg_a, params, u(1:2), body, ground_data);
[leg_b_leqddot, leg_b_lddot, leg_b_thddotabs, reaction_force_b, reaction_torque_b] ...
    = leg_dynamics(leg_b, params, u(3:4), body, ground_data);

% Calculate forces on body
body_gravity_force = params(12)*params(1)*[0; -1];
body_ground_force = ground_contact_model(body([1 3]) + [0; -0.1], body([2 4]), body([1 3]), ground_data);

body_force = reaction_force_a + reaction_force_b + body_gravity_force + body_ground_force;
body_torque = reaction_torque_a + reaction_torque_b;

% Calculate body derivatives
body_xddot = body_force(1)/params(1);
body_yddot = body_force(2)/params(1);
body_thddot = body_torque/params(2);

% Put leg thddotabs in relative coordinates
leg_a_thddot = leg_a_thddotabs - body_thddot;
leg_b_thddot = leg_b_thddotabs - body_thddot;

% Compose state derivative vector
dX = [X(2);  body_xddot;    X(4);  body_yddot;  X(6);  body_thddot; 
      X(8);  leg_a_leqddot; X(10); leg_a_lddot; X(12); leg_a_thddot;
      X(14); leg_b_leqddot; X(16); leg_b_lddot; X(18); leg_b_thddot];
0;


function leg = leg_kinematics(X_leg, body)
% Calculate lengths, derivatives, etc
leg = zeros(12, 1);
leg(1:6) = X_leg(1:6); % th, leg, l, and derivatives of each
leg(11:12) = [sin(leg(5) + body(5)); -cos(leg(5) + body(5))]; % leg direction unit vector
leg([7 9]) = body([1 3]) + leg(3)*leg(11:12); % x and y
leg([8 10]) = leg(4)*leg(11:12) + leg(6)*[-leg(9); leg(7)]; % xdot and ydot


function [leqddot, lddot, thddotabs, body_reaction_force, body_reaction_torque] ...
    = leg_dynamics(leg, params, u_leg, body, ground)
% Forces on foot other than angle motor forces
spring_force_mag = params(4)*(leg(1) - leg(3)) + params(5)*(leg(2) - leg(4));
gravity_force = params(12)*params(3)*[0; -1];
ground_force = ground_contact_model(leg([7 9]), leg([8 10]), body([1 3]), ground);

% Add foot forces (excluding angle motor) and resolve into radial coordinates
foot_force_partial = spring_force_mag*leg(11:12) + gravity_force + ground_force;
axial_force_mag = foot_force_partial(1)*leg(11) + foot_force_partial(2)*leg(12);
radial_force_mag = -foot_force_partial(1)*leg(12) + foot_force_partial(2)*leg(11);

% Reaction force and torque from leg on body
body_reaction_force = -spring_force_mag*leg(11:12) + radial_force_mag*[-leg(12); leg(11)];
body_reaction_torque = -u_leg(2) + params(11)*params(10)*(params(10)*leg(6) - (params(10) - 1)*body(5));

% Second derivatives of leg state variables
mf = params(3);
Iml = params(6);
nl = params(7);
bml = params(8);
Ima = params(9);
na = params(10);
bma = params(11);
Ieffa = mf*leg(3)^2 + na^2*Ima;
leqddot = (u_leg(1)*nl - bml*nl^2*leg(2) - spring_force_mag)/(nl^2*Iml);
lddot = axial_force_mag/mf + leg(3)*leg(6)^2;
thddotabs = (u_leg(2)*na - bma*na^2*leg(6) + leg(3)*radial_force_mag)/Ieffa - 0*2*leg(4)*leg(6)/leg(3);


function ground_force = ground_contact_model(pos, vel, ref, ground_data)
% Ground contact force model
% Takes position and velocity of point that forces act on, a reference
% position used to find the correct ground intersection location, external
% forces on the point, and the ground data structure
% Returns the ground forces and a structure containing intermediate values

% Get unit direction vector from reference to point
refdir = pos - ref;
refdir = refdir/norm(refdir);

% Find location on ground that point is contacting
[xi, yi, ii] = linexpoly([ref(1); pos(1)], [ref(2); pos(2)], ground_data(:, 1), ground_data(:, 2));
if length(xi) < 1
    % No ground contact
    ground_tangent = [1; 0];
    ground_normal = [0; 1];
    depth = 0;
    ddepth = 0;
    ground_stiffness = 0;
    ground_damping = 0;
    ground_friction = 0;
else
    % Find depth into ground, speed, and ground properties at contact
    % Ground intersection geometry calculations
    depths2 = (xi - pos(1)).^2 + (yi - pos(2)).^2;
    [~, imax] = max(depths2);
    igs = ii(imax(1));
    ground_segment = [diff(ground_data(igs:igs+1, 1)); diff(ground_data(igs:igs+1, 2))];
    intersection_vector = [xi(imax) - ground_data(igs, 1); yi(imax) - ground_data(igs, 2)];
    p = (intersection_vector(1)*ground_segment(1) + intersection_vector(2)*ground_segment(2))...
        /(ground_segment(1)^2 + ground_segment(2)^2);
    
    % Ground contact properties
    ground_tangent = ground_segment/norm(ground_segment);
    ground_normal = [-ground_tangent(2); ground_tangent(1)];
    depth = sqrt(depths2(imax))*(-refdir(1)*ground_normal(1) - refdir(2)*ground_normal(2));
    ddepth = -vel(1)*ground_normal(1) - vel(2)*ground_normal(2);
    ground_stiffness = interpolate(ground_data(:, 3), igs, p);
    ground_damping = interpolate(ground_data(:, 4), igs, p);
    ground_friction = interpolate(ground_data(:, 5), igs, p);
    
    % Make sure ground normal points towards reference position
    if -refdir(1)*ground_normal(1) - refdir(2)*ground_normal(2) < 0
        ground_tangent = -ground_tangent;
        ground_normal = -ground_normal;
    end
    
    % Ramp up damping with depth
    damping_threshold = 1e-5;
    ground_damping = ground_damping*depth/(depth + damping_threshold);
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
slip_ramp_width = friction_mag*1e-8;
p = min(max(abs(ground_slip)/slip_ramp_width, 0), 1);
friction_force = -sign(ground_slip)*p*friction_mag*ground_tangent;

% Total ground force
ground_force = spring_force + friction_force;


function out = interpolate(v, i, p)
% Interpolation function for ground properties
out = v(i) + p*(v(2) - v(i));


function [xi, yi, ii] = linexpoly(x1, y1, x2, y2)
% Customized implementation of polyxpoly for codegen
dx1 = x1(2) - x1(1);
dy1 = y1(2) - y1(1);
dx2 = x2(2:end) - x2(1:end-1);
dy2 = y2(2:end) - y2(1:end-1);
dx12 = x1(1) - x2(1:end-1);
dy12 = y1(1) - y2(1:end-1);

num = dx2.*dy12 - dy2.*dx12;
den = dy2.*dx1 - dx2.*dy1;

sa = num./den;
sb = (dx12 + sa.*dx1)./dx2;

ii = find(den ~= 0 & sa >= 0 & sa < 1 & sb >= 0 & sb < 1);
xi = x1(1) + sa(ii).*dx1;
yi = y1(1) + sa(ii).*dy1;
