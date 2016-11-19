function [dX, ext] = biped_dynamics(X, u, ext, robot, terrain)
% BIPED_DYNAMICS Calculate the biped state derivatives, taking ground contact
% and external forces into account.

% Transform into motor-side torques and clamp
u.right.l_eq = clamp(u.right.l_eq / robot.length.motor.ratio, -robot.length.motor.torque, robot.length.motor.torque);
u.right.theta_eq = clamp(u.right.theta_eq / robot.angle.motor.ratio, -robot.angle.motor.torque, robot.angle.motor.torque);
u.left.l_eq = clamp(u.left.l_eq / robot.length.motor.ratio, -robot.length.motor.torque, robot.length.motor.torque);
u.left.theta_eq = clamp(u.left.theta_eq / robot.angle.motor.ratio, -robot.angle.motor.torque, robot.angle.motor.torque);

% Get hardstop torques
hardstops.right = hardstop_torques(X.right, robot);
hardstops.left  = hardstop_torques(X.left,  robot);

% Add hardstop torques to control torques
u.right.l_eq     = u.right.l_eq     + hardstops.right.l_eq;
u.right.theta_eq = u.right.theta_eq + hardstops.right.theta_eq;
u.left.l_eq      = u.left.l_eq      + hardstops.left.l_eq;
u.left.theta_eq  = u.left.theta_eq  + hardstops.left.theta_eq;

% Add forces from ground contact and gravity to any other external forces
foot_state_right = foot_state(X.right, X.body);
foot_state_left  = foot_state(X.left,  X.body);
foot_force_right = ground_contact(foot_state_right, robot, terrain);
foot_force_left  = ground_contact(foot_state_left,  robot, terrain);
ext.body.y = ext.body.y - robot.gravity * robot.body.mass;
ext.right.x = ext.right.x + foot_force_right.x;
ext.right.y = ext.right.y + foot_force_right.y - robot.gravity * robot.foot.mass;
ext.left.x  = ext.left.x  + foot_force_left.x;
ext.left.y  = ext.left.y  + foot_force_left.y  - robot.gravity * robot.foot.mass;

% Get state derivatives from equations of motion
dX = biped_eom(X, u, ext, robot);


function t = hardstop_torques(leg, env)
% HARDSTOP_TORQUES Compute hardstop forces acting as motor torques.

% Find how far length and angle are beyond the hardstops
l_eq_over = leg.l_eq - ...
    clamp(leg.l_eq, env.length.hardstop.min, env.length.hardstop.max);
theta_eq_over = leg.theta_eq - ...
    clamp(leg.theta_eq, env.angle.hardstop.min, env.angle.hardstop.max);

% Get parameters used to fade in the derivative term near the hardstops
l_eq_dfade = outside_fade(leg.l_eq, env.length.hardstop.min, ...
    env.length.hardstop.max, env.length.hardstop.dfade);
theta_eq_dfade = outside_fade(leg.theta_eq, env.angle.hardstop.min, ...
    env.angle.hardstop.max, env.angle.hardstop.dfade);

% Spring + damper hardstops
t.l_eq = -(l_eq_over * env.length.hardstop.kp) - ...
    (leg.dl_eq * l_eq_dfade * env.length.hardstop.kd);
t.theta_eq = -(theta_eq_over * env.angle.hardstop.kp) - ...
    (leg.dtheta_eq * theta_eq_dfade * env.angle.hardstop.kd);

% Limit hardstop forces
t.l_eq = clamp(t.l_eq, -env.length.hardstop.fmax, env.length.hardstop.fmax);
t.theta_eq = clamp(t.theta_eq, -env.length.hardstop.fmax, env.length.hardstop.fmax);


function s = foot_state(leg, body)
% FOOT_STATE Compute the position and velocity of the point foot.

% Coordinate transformation parameters
theta_abs = leg.theta + body.theta;
dtheta_abs = leg.dtheta + body.dtheta;
l.x =  sin(theta_abs);
l.y = -cos(theta_abs);
theta.x = -l.y;
theta.y =  l.x;

% Use leg kinematics to get foot position and velocity
s.x = body.x + (leg.l * l.x);
s.y = body.y + (leg.l * l.y);
s.dx = body.dx + (leg.dl * l.x) + (leg.l * dtheta_abs * theta.x);
s.dy = body.dy + (leg.dl * l.y) + (leg.l * dtheta_abs * theta.y);


function f = ground_contact(point, robot, terrain)
% GROUND_CONTACT Compute forces resulting from pointwise ground contact.

npts = numel(terrain.height);
x = linspace(terrain.xstart, terrain.xend, npts);

% Find the point on the ground closest to the point to test
min_dist2 = inf;
min_p = 0;
min_x_line = 0;
min_y_line = 0;
min_seg_length2 = 0;
min_index = 1;
for i = 1:npts - 1
    xg = x(i);
    yg = terrain.height(i);
    dxg = x(i + 1) - xg;
    dyg = terrain.height(i + 1) - yg;
    
    % Take dot product to project test point onto line, then normalize with the
    % segment length squared and clamp to keep within line segment bounds
    dot_product = (point.x - xg) * dxg + (point.y - yg) * dyg;
    seg_length2 = (dxg * dxg) + (dyg * dyg);
    p = clamp(dot_product / seg_length2, 0, 1);
    
    % Nearest point on the line segment to the test point
    x_line = xg + (p * dxg);
    y_line = yg + (p * dyg);
    
    % Squared distance from line point to test point
    dist2 = ((point.x - x_line) * (point.x - x_line)) + ...
        ((point.y - y_line) * (point.y - y_line));
    
    % If this is a new minimum, save values
    % Ignore segments with zero length
    if dist2 < min_dist2 && seg_length2 > 0
        min_dist2 = dist2;
        min_p = p;
        min_x_line = x_line;
        min_y_line = y_line;
        min_seg_length2 = seg_length2;
        min_index = i;
    end
end

% Check whether point is on the ground side (right hand side) of the line
% If not, return immediately with zero ground reaction force
dxg = x(min_index + 1) - x(min_index);
dyg = terrain.height(min_index + 1) - terrain.height(min_index);
dxp = point.x - x(min_index);
dyp = point.y - terrain.height(min_index);
cross_product = (dxg * dyp) - (dyg * dxp);
if cross_product > 0.0
    f.x = 0;
    f.y = 0;
    return;
end

% If the point is a vertex, also check the next line
if min_p == 1.0 && min_index < npts - 1
    dxg = x(min_index + 2) - x(min_index + 1);
    dyg = terrain.height(min_index + 2) - terrain.height(min_index + 1);
    dxp = point.x - x(min_index + 1);
    dyp = point.y - terrain.height(min_index + 1);
    cross_product = (dxg * dyp) - (dyg * dxp);
    if cross_product > 0.0
        f.x = 0;
        f.y = 0;
        return;
    end
end

% If execution reaches here, the point is in the ground
% Note that if the test point is outside the bounds of the
% polyline, it is handled incorrectly

% Get normal and tangent basis vectors
% NOTE: Normal is into ground, tangent is 90 deg CCW from normal
depth = sqrt(min_dist2);
if (min_p == 0.0 || min_p == 1.0) && depth > 0
    % Special case for corners -- normal is aligned with vector
    % from test point to corner
    normal_x = -(point.x - min_x_line) / depth;
    normal_y = -(point.y - min_y_line) / depth;
    tangent_x = normal_y;
    tangent_y = -normal_x;
else
    % Typical case -- use segment direction for tangent
    seg_length = sqrt(min_seg_length2);
    tangent_x = dxg / seg_length;
    tangent_y = dyg / seg_length;
    normal_x = -tangent_y;
    normal_y = tangent_x;
end

% Get derivative of depth
ddepth = (-normal_x * point.dx) + (-normal_y * point.dy);

% Damping adjustment factor
damping_factor = depth / (depth + robot.ground.damping_depth);

% Normal force (spring + damper) should only be positive upwards
normal_force = max((depth * terrain.stiffness) + (ddepth * damping_factor * terrain.damping), 0);

% Tangent force (friction) before finding sign and smoothing discontinuity
friction_max = terrain.friction * normal_force;
tangent_velocity = (tangent_x * point.dx) + (tangent_y * point.dy);
viscous_friction_factor = clamp(tangent_velocity / (friction_max * robot.ground.slip_ramp), -1, 1);
tangent_force = -viscous_friction_factor * friction_max;

f.x = (normal_x * normal_force) + (tangent_x * tangent_force);
f.y = (normal_y * normal_force) + (tangent_y * tangent_force);


function dX = biped_eom(X, u, ext, env)
% BIPED_EOM Use final motor and external forces to calculate state deriatives.

% Calculate motor gap torques, taking damping into account
angle_motor_gap_torque_right = u.right.theta_eq - ...
    (env.angle.motor.damping * X.right.dtheta_eq * env.angle.motor.ratio);
length_motor_gap_torque_right = u.right.l_eq - ...
    (env.length.motor.damping * X.right.dl_eq * env.length.motor.ratio);
angle_motor_gap_torque_left = u.left.theta_eq - ...
    (env.angle.motor.damping * X.left.dtheta_eq * env.angle.motor.ratio);
length_motor_gap_torque_left = u.left.l_eq - ...
    (env.length.motor.damping * X.left.dl_eq * env.length.motor.ratio);

% Calculate internal spring forces
length_spring_force_right = (env.length.stiffness * (X.right.l_eq - X.right.l)) + ...
    (env.length.damping * (X.right.dl_eq - X.right.dl));
angle_spring_torque_right = (env.angle.stiffness * (X.right.theta_eq - X.right.theta)) + ...
    (env.angle.damping * (X.right.dtheta_eq - X.right.dtheta));
angle_spring_force_right = angle_spring_torque_right / X.right.l;
length_spring_force_left = (env.length.stiffness * (X.left.l_eq - X.left.l)) + ...
    (env.length.damping * (X.left.dl_eq - X.left.dl));
angle_spring_torque_left = (env.angle.stiffness * (X.left.theta_eq - X.left.theta)) + ...
    (env.angle.damping * (X.left.dtheta_eq - X.left.dtheta));
angle_spring_force_left = angle_spring_torque_left / X.left.l;

% Get basis vectors for internal spring forces
% Positive when acting on the foot, negate for body
l_x_right = sin(X.right.theta + X.body.theta);
l_y_right = -cos(X.right.theta + X.body.theta);
theta_x_right = -l_y_right;
theta_y_right = l_x_right;
l_x_left = sin(X.left.theta + X.body.theta);
l_y_left = -cos(X.left.theta + X.body.theta);
theta_x_left = -l_y_left;
theta_y_left = l_x_left;

 % Forces on body
force_body_x = ext.body.x - (l_x_right * length_spring_force_right) - ...
    (theta_x_right * angle_spring_force_right) - (l_x_left * length_spring_force_left) - ...
    (theta_x_left * angle_spring_force_left);
force_body_y = ext.body.y - (l_y_right * length_spring_force_right) - ...
    (theta_y_right * angle_spring_force_right) - (l_y_left * length_spring_force_left) - ...
    (theta_y_left * angle_spring_force_left);
torque_body_theta = ext.body.theta - angle_motor_gap_torque_right - angle_motor_gap_torque_left - ...
    ((1 - 1/env.angle.motor.ratio) * (angle_spring_torque_right + angle_spring_torque_left));

% Body position derivatives
% Struct definitions must be in order
dX.body.x = X.body.dx;
dX.body.y = X.body.dy;
dX.body.theta = X.body.dtheta;

% Acceleration of body
dX.body.dx = force_body_x / env.body.mass;
dX.body.dy = force_body_y / env.body.mass;
dX.body.dtheta = torque_body_theta / env.body.inertia;

% Remaining position derivatives and velocity initialization
dX.right.l = X.right.dl;
dX.right.l_eq = X.right.dl_eq;
dX.right.theta = X.right.dtheta;
dX.right.theta_eq = X.right.dtheta_eq;
dX.right.dl        = 0;
dX.right.dl_eq     = 0;
dX.right.dtheta    = 0;
dX.right.dtheta_eq = 0;
dX.left.l = X.left.dl;
dX.left.l_eq = X.left.dl_eq;
dX.left.theta = X.left.dtheta;
dX.left.theta_eq = X.left.dtheta_eq;
dX.left.dl        = 0;
dX.left.dl_eq     = 0;
dX.left.dtheta    = 0;
dX.left.dtheta_eq = 0;

% Acceleration of leg equilibrium positions
dX.right.dtheta_eq = (angle_motor_gap_torque_right - angle_spring_torque_right / env.angle.motor.ratio) / ...
    (env.angle.motor.ratio * env.angle.motor.inertia);
dX.right.dl_eq = (length_motor_gap_torque_right - length_spring_force_right / env.length.motor.ratio) / ...
    (env.length.motor.ratio * env.length.motor.inertia);
dX.left.dtheta_eq = (angle_motor_gap_torque_left - angle_spring_torque_left / env.angle.motor.ratio) / ...
    (env.angle.motor.ratio * env.angle.motor.inertia);
dX.left.dl_eq = (length_motor_gap_torque_left - length_spring_force_left / env.length.motor.ratio) / ...
    (env.length.motor.ratio * env.length.motor.inertia);

% Convert external forces on foot to relative polar coordinate acceleration
% Gravity is included in the external forces
accel_offset_foot_x_right = ext.right.x / env.foot.mass - dX.body.dx;
accel_offset_foot_y_right = ext.right.y / env.foot.mass - dX.body.dy;
accel_foot_l_right = (length_spring_force_right / env.foot.mass) + (accel_offset_foot_x_right * l_x_right) + ...
    (accel_offset_foot_y_right * l_y_right);
accel_foot_theta_right = (angle_spring_force_right / env.foot.mass) + (accel_offset_foot_x_right * theta_x_right) + ...
    (accel_offset_foot_y_right * theta_y_right);
accel_offset_foot_x_left = ext.left.x / env.foot.mass - dX.body.dx;
accel_offset_foot_y_left = ext.left.y / env.foot.mass - dX.body.dy;
accel_foot_l_left = (length_spring_force_left / env.foot.mass) + (accel_offset_foot_x_left * l_x_left) + ...
    (accel_offset_foot_y_left * l_y_left);
accel_foot_theta_left = (angle_spring_force_left / env.foot.mass) + (accel_offset_foot_x_left * theta_x_left) + ...
    (accel_offset_foot_y_left * theta_y_left);

% Acceleration of actual leg positions
dtheta_abs_right = X.right.dtheta + X.body.dtheta;
dX.right.dl = accel_foot_l_right + (X.right.l * dtheta_abs_right * dtheta_abs_right);
dX.right.dtheta = (accel_foot_theta_right - (2 * X.right.dl * dtheta_abs_right)) / X.right.l - dX.body.dtheta;
dtheta_abs_left = X.left.dtheta + X.body.dtheta;
dX.left.dl = accel_foot_l_left + (X.left.l * dtheta_abs_left * dtheta_abs_left);
dX.left.dtheta = (accel_foot_theta_left - (2 * X.left.dl * dtheta_abs_left)) / X.left.l - dX.body.dtheta;


function out = clamp(x, lower, upper)
% CLAMP Constrain the value to be within the given bounds.
out = min(max(x, lower), upper);


function out = outside_fade(x, lower, upper, fade)
% OUTSIDE_FADE Returns 1 when x is outside the given bounds, 0 when far inside
% the bounds, and a smooth fade from 0 to 1 when approaching the bounds.
x_over = x - clamp(x, lower + fade, upper - fade);
out = clamp(abs(x_over / fade), 0, 1);
