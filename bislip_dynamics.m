function [dY, body, leg_a, leg_b] = bislip_dynamics(Y, u, params, ground_data)
% Y: [body_x; body_y; body_th; com_xdot; body_ydot; body_thdot;
%     foot_a_x; foot_a_y; foot_a_xdot; foot_a_ydot;
%     foot_b_x; foot_b_y; foot_b_xdot; foot_b_ydot]

%#codegen
assert(isa(Y, 'double') && all(size(Y) == [14 1]) && isreal(Y));
assert(isa(u, 'double') && all(size(u) == [4 1]) && isreal(u));
assert(isa(params, 'double') && all(size(params) == [6 1]) && isreal(params));
assert(isa(ground_data, 'double') && size(ground_data, 2) == 5 && isreal(ground_data));

% Environment
gravity =          params(6);
ground.x =         ground_data(:, 1);
ground.y =         ground_data(:, 2);
ground.stiffness = ground_data(:, 3);
ground.damping =   ground_data(:, 4);
ground.friction =  ground_data(:, 5);

% Kinematics
[body, leg_a, leg_b] = bislip_kinematics(Y, params);

% Control inputs
leg_a.length_eq = u(1);
leg_b.length_eq = u(2);
leg_a.torque =    u(3);
leg_b.torque =    u(4);

% Compute quantities for each leg
leg_a = leg_dynamics(leg_a, body, ground, gravity);
leg_b = leg_dynamics(leg_b, body, ground, gravity);

% Calculate forces on body (other than ground reaction)
body.spring_a_force = -leg_a.spring_force;
body.spring_b_force = -leg_b.spring_force;
body.gravity_force = gravity*body.mass*[0; -1];

% Ground reaction force for body
nonground_force = body.spring_a_force + body.spring_b_force + body.gravity_force;
body.ground_force = ground_contact_model(body.pos + [0; -0.1], body.dpos, body.pos, nonground_force, ground);

body.force = nonground_force + body.ground_force;
body.torque = -leg_a.torque + -leg_b.torque;

% Compose state derivative vector
dY = [body.dpos; body.dth; body.force/body.mass; body.torque/body.inertia;
    leg_a.dfoot; leg_a.foot_force/leg_a.mass;
    leg_b.dfoot; leg_b.foot_force/leg_b.mass];


function leg = leg_dynamics(leg, body, ground, gravity)

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
leg.ground_force = ground_contact_model(leg.foot, leg.dfoot, body.pos, nonground_force, ground);

% Net forces on foot
leg.foot_force = nonground_force + leg.ground_force;


function ground_force = ground_contact_model(pos, vel, ref, external_force, ground)
% Ground contact force model
% Takes position and velocity of point that forces act on, a reference
% position used to find the correct ground intersection location, external
% forces on the point, and the ground data structure
% Returns the ground forces and a structure containing intermediate values

% Get unit direction vector from reference to point
gc = makegc();
gc.refdir = pos - ref;
gc.refdir = gc.refdir/norm(gc.refdir);

% Find location on ground that point is contacting
[xi, yi, ii] = polyxpoly_internal([ref(1) pos(1)], [ref(2) pos(2)], ground.x, ground.y);
if length(xi) >= 1
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
slip_ramp_width = gc.friction_mag*1e-8;
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


function [xi, yi, ii] = polyxpoly_internal(x1, y1, x2, y2)
% Reimplementation of polyxpoly for codegen
assert(isa(x1, 'double') && isa(y1, 'double') &&length(x1) == length(y1) && isreal(x1) && isreal(y1));
assert(isa(x2, 'double') && isa(y2, 'double') &&length(x2) == length(y2) && isreal(x2) && isreal(y2));

xi = zeros(0, 1);
yi = zeros(0, 1);
ii = zeros(0, 2);

for i = 1:length(x1)-1
    for j = 1:length(x2)-1
        v12 = [x1(i+1) y1(i+1)] - [x1(i) y1(i)];
        v34 = [x2(j+1) y2(j+1)] - [x2(j) y2(j)];
        v31 = [x1(i) y1(i)] - [x2(j) y2(j)];
        
        num = v34(1)*v31(2) - v34(2)*v31(1);
        den = v34(2)*v12(1) - v34(1)*v12(2);
        
        if den == 0
            continue
        end
        
        sa = num/den;
        sb = (v31(1) + sa*v12(1)) / v34(1);
        if sa >= 0 && sa < 1 && sb >= 0 && sb < 1
            p = [x1(i) y1(i)] + sa*v12;
            xi = [xi; p(1)];
            yi = [yi; p(2)];
            ii = [ii; i j];
        end
    end
end


function gc = makegc()
% Define fields of ground contact struct
gc.refdir = [1; 0];
gc.ground_tangent = [1; 0];
gc.ground_normal = [0; 1];
gc.depth = 0;
gc.ddepth = 0;
gc.ground_stiffness = 0;
gc.ground_damping = 0;
gc.ground_friction = 0;
gc.spring_force = [0; 0];
gc.friction_mag = 0;
gc.tangential_force = 0;
gc.ground_slip = 0;
gc.p = 0;
gc.friction_force = [0; 0];
