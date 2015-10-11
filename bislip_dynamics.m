function [dY, body, leg_a, leg_b] = bislip_dynamics(Y, u, params, ground_data)
% Y: [body_x; body_y; body_th; com_xdot; body_ydot; body_thdot;
%     foot_a_x; foot_a_y; foot_a_xdot; foot_a_ydot;
%     foot_b_x; foot_b_y; foot_b_xdot; foot_b_ydot]

% Physical parameters
body.mass =       params(1);
body.inertia =    params(2);
leg_a.mass =      params(3);
leg_b.mass =      params(3);
leg_a.stiffness = params(4);
leg_b.stiffness = params(4);
leg_a.damping =   params(5);
leg_b.damping =   params(5);
gravity =         params(6);

% Environment
ground.x =         ground_data(:, 1);
ground.y =         ground_data(:, 2);
ground.stiffness = ground_data(:, 3);
ground.damping =   ground_data(:, 4);
ground.friction =  ground_data(:, 5);

% Control inputs
leg_a.length_eq = u(1);
leg_b.length_eq = u(2);
leg_a.torque =    u(3);
leg_b.torque =    u(4);

% Break Y out
body.pos =    Y(1:2);
body.th =     Y(3);
body.dpos =   Y(4:5);
body.dth =    Y(6);
leg_a.foot =  Y(7:8);
leg_a.dfoot = Y(9:10);
leg_b.foot =  Y(11:12);
leg_b.dfoot = Y(13:14);

% Compute quantities for each leg
leg_a = legcalcs(leg_a, body, ground, gravity);
leg_b = legcalcs(leg_b, body, ground, gravity);

% Calculate forces on body (other than ground reaction)
body.spring_a_force = -leg_a.spring_force;
body.spring_b_force = -leg_b.spring_force;
body.gravity_force = gravity*body.mass*[0; -1];

% Ground reaction force for body
nonground_force = body.spring_a_force + body.spring_b_force + body.gravity_force;
[body.ground_force, body.dpos, body.ground_contact_data] = ...
    ground_contact_model(body.pos + [0; -0.1], body.dpos, body.pos, nonground_force, ground);

body.force = nonground_force + body.ground_force;
body.torque = -leg_a.torque + -leg_b.torque;

% Compose state derivative vector
dY = [body.dpos; body.dth; body.force/body.mass; body.torque/body.inertia;
    leg_a.dfoot; leg_a.foot_force/leg_a.mass;
    leg_b.dfoot; leg_b.foot_force/leg_b.mass];


function leg = legcalcs(leg, body, ground, gravity)
% Calculate lengths, derivatives, etc
leg.vec = leg.foot - body.pos;
leg.length = norm(leg.vec);
if leg.length ~= 0
    leg.direction = leg.vec/leg.length;
else
    leg.direction = [0; -1];
end
leg.angle = atan2(leg.direction(1), -leg.direction(2));
leg.dvec = leg.dfoot - body.dpos;
leg.dlength = dot(leg.dvec, leg.direction);

% Forces acting on foot (other than ground reaction)
leg.spring_force = (leg.stiffness*(leg.length_eq - leg.length) - leg.damping*leg.dlength)*leg.direction;
if leg.length ~=0
    leg.motor_force = leg.torque/leg.length*ccw90(leg.direction);
else
    leg.motor_force = [0; 0];
end
leg.gravity_force = gravity*leg.mass*[0; -1];

% Get ground forces from ground contact model
nonground_force = leg.spring_force + leg.motor_force + leg.gravity_force;
[leg.ground_force, leg.dfoot, leg.ground_contact_data] = ...
    ground_contact_model(leg.foot, leg.dfoot, body.pos, nonground_force, ground);

% Net forces on foot
leg.foot_force = nonground_force + leg.ground_force;


function [ground_force, vel, gc] = ground_contact_model(pos, vel, ref, external_force, ground)
% Ground contact force model
% Takes position and velocity of point that forces act on, a reference
% position used to find the correct ground intersection location, external
% forces on the point, and the ground data structure
% Returns the ground forces and a structure containing intermediate values

% Get unit direction vector from reference to point
gc.refdir = pos - ref;
gc.refdir = gc.refdir/norm(gc.refdir);

% Find location on ground that point is contacting
[xi, yi, ii] = polyxpoly([ref(1) pos(1)], [ref(2) pos(2)], ground.x, ground.y);
if length(xi) < 1
    % No contact
    gc.ground_tangent = [1; 0];
    gc.ground_normal = [0; 1];
    gc.depth = 0;
    gc.ddepth = 0;
    gc.ground_stiffness = 0;
    gc.ground_damping = 0;
    gc.ground_friction = 0;
else
    % Find depth into ground, speed, and ground properties at contact
    % Ground intersection geometry calculations
    depths = sqrt((xi - pos(1)).^2 + (yi - pos(2)).^2);
    [~, imax] = max(depths);
    igs = ii(imax, 2);
    ground_segment = [diff(ground.x(igs:igs+1)); diff(ground.y(igs:igs+1))];
    intersection_vector = [xi(imax) - ground.x(igs); yi(imax) - ground.y(igs)];
    p = dot(intersection_vector, ground_segment)/norm(ground_segment)^2;
    
    % Ground contact properties
    gc.ground_tangent = ground_segment/norm(ground_segment);
    gc.ground_normal = ccw90(gc.ground_tangent);
    gc.depth = depths(imax)*dot(gc.refdir, -gc.ground_normal);
    gc.ddepth = dot(vel, -gc.ground_normal);
    gc.ground_stiffness = interpolate(ground.stiffness, igs, p);
    gc.ground_damping = interpolate(ground.damping, igs, p);
    gc.ground_friction = interpolate(ground.friction, igs, p);
    
    % Make sure ground normal points towards reference position
    if dot(-gc.refdir, gc.ground_normal) < 0
        gc.ground_tangent = -gc.ground_tangent;
        gc.ground_normal = -gc.ground_normal;
    end
    
    % Ramp up damping with depth
    damping_threshold = 1e-5;
    gc.ground_damping = gc.ground_damping*gc.depth/(gc.depth + damping_threshold);
end

% Ground reaction force from spring-damper system
% Acts nomral to the ground
gc.spring_force = gc.ground_normal*max(gc.depth*gc.ground_stiffness + gc.ddepth*gc.ground_damping, 0);

% Friction magnitude is proportional to component of ground spring force
% normal to ground
gc.friction_mag = gc.ground_friction*dot(gc.spring_force, gc.ground_normal);

% Calculate friction, replacing the jump discontinuity due to the sign
% function with a continuous ramp
gc.tangential_force = dot(external_force, gc.ground_tangent);
gc.ground_slip = dot(vel, gc.ground_tangent);
slip_ramp_width = gc.friction_mag*1e-3;
gc.p = min(max(abs(gc.ground_slip)/slip_ramp_width, 0), 1);
gc.friction_force = -sign(gc.ground_slip)*gc.p*gc.friction_mag*gc.ground_tangent;

% Total ground force
ground_force = gc.spring_force + gc.friction_force;


function out = ccw90(v)
% Rotate vector 90 degrees ccw
out = [-v(2); v(1)];


function out = interpolate(v, i, p)
% Interpolation function for ground properties
out = v(i) + p*(v(i+1) - v(i));

