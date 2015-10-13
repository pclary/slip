function [dX, body, leg_a, leg_b] = bislip_dynamics(X, u, params, ground_data)
% Y: [body_x;   body_y;   body_xdot;   body_ydot;   body_th; body_thdot;
%     foot_a_x; foot_a_y; foot_a_xdot; foot_a_ydot;
%     foot_b_x; foot_b_y; foot_b_xdot; foot_b_ydot]

% Physical parameters
body_params = params(1:2); % mass; inertia
leg_params = params(3:5); % mass, stiffness, damping
gravity = params(6);

% Kinematics
[body, leg_a, leg_b] = bislip_kinematics(X);

% Compute quantities for each leg
[leg_a_foot_force, leg_a_spring_force, leg_a_motor_force] ...
    = leg_dynamics(leg_a, leg_params, u(1:2), body(1:2), ground_data, gravity);
[leg_b_foot_force, leg_b_spring_force, leg_b_motor_force] ...
    = leg_dynamics(leg_b, leg_params, u(3:4), body(1:2), ground_data, gravity);

% Calculate forces on body
body_spring_a_force = -leg_a_spring_force;
body_spring_b_force = -leg_b_spring_force;
body_motor_a_force = -leg_a_motor_force;
body_motor_b_force = -leg_b_motor_force;
body_gravity_force = gravity*body_params(1)*[0; -1];
body_ground_force = ground_contact_model(body(1:2) + [0; -0.1], body(3:4), body(1:2), ground_data);

body_force = body_spring_a_force + body_spring_b_force ...
    + body_motor_a_force + body_motor_b_force ...
    + body_gravity_force + body_ground_force;
body_torque = -u(1) + -u(3);

% Compose state derivative vector
dX = [body(3:4);  body_force/body_params(1);      body(6); body_torque/body_params(2);
      leg_a(3:4); leg_a_foot_force/leg_params(1);
      leg_b(3:4); leg_b_foot_force/leg_params(1)];


function [body, leg_a, leg_b] = bislip_kinematics(X)
% body: [x; y; xdot; ydot; th; thdot]
% leg: [x; y; xdot; ydot; th; thdot; l; ldot; xdir; ydir]

% Break Y out
body = X(1:6);
foot_a = X(7:10);
foot_b = X(11:14);

leg_a = leg_kinematics(foot_a, body);
leg_b = leg_kinematics(foot_b, body);


function leg = leg_kinematics(foot, body)
% Calculate lengths, derivatives, etc
leg = zeros(10, 1);
leg(1:4) = foot; % position, velocity
vec = foot(1:2) - body(1:2);
dvec = foot(3:4) - body(3:4);
leg(7) = sqrt(vec(1)^2 + vec(2)^2); % length
if leg(7) ~= 0 % length ~= 0
    leg(9:10) = vec/leg(7); % direction
    leg(6) = (-leg(10)*dvec(1) + leg(9)*dvec(2))/leg(7); % thdot
else
    leg(9:10) = [0; -1]; % direction
    leg(6) = 0; % thdot
end
leg(5) = atan2(leg(9), -leg(10)); % th
leg(8) = dvec(1)*leg(9) + dvec(2)*leg(10); % lengthdot


function [foot_force, spring_force, motor_force] ...
    = leg_dynamics(leg, leg_params, u_leg, body_pos, ground, gravity)

% Forces acting on foot (other than ground reaction)
spring_force = (leg_params(2)*(u_leg(2) - leg(7)) - leg_params(3)*leg(8))*leg(9:10);
if leg(7) ~=0
    motor_force = u_leg(1)/leg(7)*[-leg(10); leg(9)];
else
    motor_force = [0; 0];
end
gravity_force = gravity*leg_params(1)*[0; -1];
ground_force = ground_contact_model(leg(1:2), leg(3:4), body_pos, ground);

% Net forces on foot
foot_force = spring_force + motor_force + gravity_force + ground_force;


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
    p = (intersection_vector(1)*ground_segment(1) + intersection_vector(2)*ground_segment(2))/(ground_segment(1)^2 + ground_segment(2)^2);
    
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
