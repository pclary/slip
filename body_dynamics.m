function [dX_body, ext] = body_dynamics(X, u, ext, env)
% BODY_DYNAMICS Calculate derivatives of the body state.

% Get leg forces
f.right.l_eq = u.right.l_eq.torque + u.right.l_eq.kp * (u.right.l_eq.target - X.right.l_eq) + ...
    u.right.l_eq.kd * (u.right.l_eq.dtarget - X.right.dl_eq);
f.right.theta_eq = u.right.theta_eq.torque + u.right.theta_eq.kp * (u.right.theta_eq.target - X.right.theta_eq) + ...
    u.right.theta_eq.kd * (u.right.theta_eq.dtarget - X.right.dtheta_eq);
f.left.l_eq = u.left.l_eq.torque + u.left.l_eq.kp * (u.left.l_eq.target - X.left.l_eq) + ...
    u.left.l_eq.kd * (u.left.l_eq.dtarget - X.left.dl_eq);
f.left.theta_eq = u.left.theta_eq.torque + u.left.theta_eq.kp * (u.left.theta_eq.target - X.left.theta_eq) + ...
    u.left.theta_eq.kd * (u.left.theta_eq.dtarget - X.left.dtheta_eq);

% No net force on body if pushing against hardstops
if X.right.l <= env.length.hardstop.min || X.right.l >= env.length.hardstop.max
    f.right.l_eq = 0;
end
if X.right.theta <= env.angle.hardstop.min || X.right.theta >= env.angle.hardstop.max
    f.right.theta_eq = 0;
end
if X.left.l <= env.length.hardstop.min || X.left.l >= env.length.hardstop.max
    f.left.l_eq = 0;
end
if X.left.theta <= env.angle.hardstop.min || X.left.theta >= env.angle.hardstop.max
    f.left.theta_eq = 0;
end

% Add force from gravity
ext.body.y = ext.body.y - env.gravity * env.body.mass;

% Get state derivatives from equations of motion
% Only the body derivatives are actually calculated for the simplified model
[dX_body, F_legs] = body_eom(X, f, ext.body, env);

% Add forces from leg on body to output for scope display
ext.right = F_legs.right;
ext.left  = F_legs.left;


function [dX_body, F_legs] = body_eom(X, F_motors, ext_body, env)
% BODY_EOM Use spring and external forces to find body state derivatives.

% Get basis vectors for internal spring forces
% Positive when acting on the foot, negate for body
l_x_right =  sin(X.right.theta + X.body.theta);
l_y_right = -cos(X.right.theta + X.body.theta);
theta_x_right = -l_y_right;
theta_y_right =  l_x_right;
l_x_left =  sin(X.left.theta + X.body.theta);
l_y_left = -cos(X.left.theta + X.body.theta);
theta_x_left = -l_y_left;
theta_y_left =  l_x_left;

% Forces from legs on the body
F_legs.right.x = -(l_x_right * F_motors.right.l_eq) - (theta_x_right * F_motors.right.theta_eq);
F_legs.right.y = -(l_y_right * F_motors.right.l_eq) - (theta_y_right * F_motors.right.theta_eq);
F_legs.left.x  = -(l_x_left * F_motors.left.l_eq)   - (theta_x_left * F_motors.left.theta_eq);
F_legs.left.y  = -(l_y_left * F_motors.left.l_eq)   - (theta_y_left * F_motors.left.theta_eq);

 % Forces on body
force_body_x = ext_body.x + F_legs.right.x + F_legs.left.x;
force_body_y = ext_body.y + F_legs.right.y + F_legs.left.y;

% Body position derivatives
% Struct definitions must be in order
dX_body.x = X.body.dx;
dX_body.y = X.body.dy;
dX_body.theta = 0;

% Acceleration of body
dX_body.dx = force_body_x / env.body.mass;
dX_body.dy = force_body_y / env.body.mass;
dX_body.dtheta = 0;
