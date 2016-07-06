function [X_right, X_left] = leg_dynamics(X_body_new, X, u, env, ground_data, Ts)
% LEG_DYNAMICS Directly compute new leg states.

% Initialize output structs
X_right = X.right;
X_left  = X.left;

% Calculate previous foot state
[pg_right_last, inground_right_last] = ground_contact(foot_state(X.right, X.body), ground_data);
[pg_left_last,  inground_left_last]  = ground_contact(foot_state(X.left,  X.body), ground_data);

% Calculate new foot state, assuming no ground collisions
right_l_target = clamp(u.right.l_eq.target + zeronan(u.right.l_eq.torque / u.right.l_eq.kp), ...
    env.length.hardstop.min, env.length.hardstop.max);
right_theta_target = clamp(u.right.theta_eq.target + zeronan(u.right.theta_eq.torque / u.right.theta_eq.kp), ...
    env.angle.hardstop.min, env.angle.hardstop.max);
X_right.l_eq     = right_l_target;
X_right.l        = right_l_target;
X_right.theta_eq = right_theta_target;
X_right.theta    = right_theta_target;
left_l_target = clamp(u.left.l_eq.target + zeronan(u.left.l_eq.torque / u.left.l_eq.kp), ...
    env.length.hardstop.min, env.length.hardstop.max);
left_theta_target = clamp(u.left.theta_eq.target + zeronan(u.left.theta_eq.torque / u.left.theta_eq.kp), ...
    env.angle.hardstop.min, env.angle.hardstop.max);
X_left.l_eq     = left_l_target;
X_left.l        = left_l_target;
X_left.theta_eq = left_theta_target;
X_left.theta    = left_theta_target;
[pg_right_new, inground_right_new] = ground_contact(foot_state(X_right, X_body_new), ground_data);
[pg_left_new,  inground_left_new]  = ground_contact(foot_state(X_left,  X_body_new), ground_data);

% Modify leg state in case of ground collision
if inground_right_new
    if inground_right_last
        X_right = leg_state(pg_right_last, X_body_new);
    else
        X_right = leg_state(pg_right_new, X_body_new);
    end
    
    % Equalize leg springs and PD controllers
    X_right.l_eq = right_l_target + (X_right.l - right_l_target) * ...
        (env.length.stiffness / (env.length.stiffness + u.right.l_eq.kp));
    X_right.theta_eq = right_theta_target + (X_right.theta - right_theta_target) * ...
        (env.angle.stiffness / (env.angle.stiffness + u.right.theta_eq.kp));
end
if inground_left_new
    if inground_left_last
        X_left = leg_state(pg_left_last, X_body_new);
    else
        X_left = leg_state(pg_left_new, X_body_new);
    end
    
    % Equalize leg springs and PD controllers
    X_left.l_eq = left_l_target + (X_left.l - left_l_target) * ...
        (env.length.stiffness / (env.length.stiffness + u.left.l_eq.kp));
    X_left.theta_eq = left_theta_target + (X_left.theta - left_theta_target) * ...
        (env.angle.stiffness / (env.angle.stiffness + u.left.theta_eq.kp));
end

% Leg derivatives
X_right.dl        = (X_right.l - X.right.l) / Ts;
X_right.dl_eq     = (X_right.l_eq - X.right.l_eq) / Ts;
X_right.dtheta    = (X_right.theta - X.right.theta) / Ts;
X_right.dtheta_eq = (X_right.theta_eq - X.right.theta_eq) / Ts;
X_left.dl        = (X_left.l - X.left.l) / Ts;
X_left.dl_eq     = (X_left.l_eq - X.left.l_eq) / Ts;
X_left.dtheta    = (X_left.theta - X.left.theta) / Ts;
X_left.dtheta_eq = (X_left.theta_eq - X.left.theta_eq) / Ts;



function foot = foot_state(leg, body)
% FOOT_STATE Compute the position and velocity of the point foot.

% Coordinate transformation parameters
theta_abs = leg.theta + body.theta;
l.x =  sin(theta_abs);
l.y = -cos(theta_abs);

% Use leg kinematics to get foot position and velocity
foot.x = body.x + (leg.l * l.x);
foot.y = body.y + (leg.l * l.y);


function leg = leg_state(foot, body)
% LEG_STATE Inverse of foot_state.

% Leg vector
l_x = foot.x - body.x;
l_y = foot.y - body.y;

% Transform to polar
l = sqrt(l_x^2 + l_y^2);
theta = atan2(l_x, -l_y) - body.theta;

% Assemble output struct
leg.l         = l;
leg.l_eq      = l;
leg.theta     = theta;
leg.theta_eq  = theta;
leg.dl        = 0;
leg.dl_eq     = 0;
leg.dtheta    = 0;
leg.dtheta_eq = 0;


function [pg, inground] = ground_contact(point, ground_data)
% GROUND_CONTACT Find nearest ground point and whether the test point is in
% the ground.

% Find the point on the ground closest to the point to test
min_dist2 = inf;
min_p = 0;
min_x_line = 0;
min_y_line = 0;
min_index = 1;
for i = 1:size(ground_data, 1) - 1
    xg = ground_data(i, 1);
    yg = ground_data(i, 2);
    dxg = ground_data(i + 1, 1) - xg;
    dyg = ground_data(i + 1, 2) - yg;
    
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
        min_index = i;
    end
end

% Closest point on ground surface to test point
pg.x = min_x_line;
pg.y = min_y_line;

% Check whether point is on the ground side (right hand side) of the line
% If not, return immediately with zero ground reaction force
dxg = ground_data(min_index + 1, 1) - ground_data(min_index, 1);
dyg = ground_data(min_index + 1, 2) - ground_data(min_index, 2);
dxp = point.x - ground_data(min_index, 1);
dyp = point.y - ground_data(min_index, 2);
cross_product = (dxg * dyp) - (dyg * dxp);
if cross_product > 1e-6 % Use a small value instead of zero to deal with floating point issues
    inground = false;
    return;
end

% If the point is a vertex, also check the next line
if min_p == 1.0 && min_index < size(ground_data, 1) - 1
    dxg = ground_data(min_index + 2, 1) - ground_data(min_index + 1, 1);
    dyg = ground_data(min_index + 2, 2) - ground_data(min_index + 1, 2);
    dxp = point.x - ground_data(min_index + 1, 1);
    dyp = point.y - ground_data(min_index + 1, 2);
    cross_product = (dxg * dyp) - (dyg * dxp);
    if cross_product > 1e-6
        inground = false;
        return;
    end
end

% Otherwise, the point is in the ground
inground = true;


function out = clamp(x, lower, upper)
% CLAMP Constrain the value to be within the given bounds.
out = min(max(x, lower), upper);


function out = zeronan(x)
% ZERONAN Replace NaN with zero.
if ~isnan(x)
    out = x;
else
    out = 0;
end
