function [u, s] = controller_step(X, s, p, Ts)

robot_weight = 3e2;
l_max = 0.8;
l_min = l_max - p.step_height;

if abs(X.body.dx / p.phase_rate) > p.max_stride * 2
     p.phase_rate = clamp(abs(X.body.dx / p.max_stride / 2), 0.1, 10);
end

% Increase leg phases with time
s.phase.right = s.phase.right + Ts * p.phase_rate;
s.phase.left  = s.phase.left  + Ts * p.phase_rate;

% Update leg targets
foot_extension = X.body.dx / (2 * p.phase_rate) + ...
    0.1 * clamp((X.body.dx - p.target_dx), -0.5, 0.5) + ...
    0.2 * (X.body.dx - s.X_laststep.body.dx);
if s.phase.right >= 1
    s.footstep_target_left = X.body.x + foot_extension;
    s.footstep_target_right = X.body.x + 2 * foot_extension;
    s.X_laststep = X;
end
if s.phase.left >= 1
    s.footstep_target_right = X.body.x + foot_extension;
    s.footstep_target_left = X.body.x + 2 * foot_extension;
    s.X_laststep = X;
end

% Limit phases to [0, 1)
s.phase.right = s.phase.right - floor(s.phase.right);
s.phase.left  = s.phase.left  - floor(s.phase.left);

% Get singleturn body angle to allow for flips
body_theta_st = mod(X.body.theta + pi, 2*pi) - pi;

% Initialize torque struct
u.right.l_eq = 0;
u.right.theta_eq = 0;
u.left.l_eq = 0;
u.left.theta_eq = 0;

% Leg length control uses a PD controlled trajectory, a feedforward term for
% weight compensation, and an energy injection term
u.right.l_eq = ...
    eval_pd(p.length_pd, s.phase.right, X.right.l_eq, X.right.dl_eq, l_min, l_max) + ...
    eval_ff(p.weight_ff, s.phase.right) * robot_weight + ...
    eval_ff(p.energy_ff, s.phase.right) * p.energy_injection;
u.left.l_eq = ...
    eval_pd(p.length_pd, s.phase.left, X.left.l_eq, X.left.dl_eq, l_min, l_max) + ...
    eval_ff(p.weight_ff, s.phase.left) * robot_weight + ...
    eval_ff(p.energy_ff, s.phase.left) * p.energy_injection;

% Angle targets are modulated by footstep target location
theta_eq_target_right = real(asin(complex((s.footstep_target_right + p.step_offset - X.body.x) / X.right.l))) - body_theta_st;
u.right.theta_eq = eval_pd(p.angle_pd, s.phase.right, ...
    X.right.theta_eq, X.right.dtheta_eq, ...
    s.X_laststep.right.theta_eq, theta_eq_target_right);

theta_eq_target_left = real(asin(complex((s.footstep_target_left + p.step_offset - X.body.x) / X.left.l))) - body_theta_st;
u.left.theta_eq = eval_pd(p.angle_pd, s.phase.left, ...
    X.left.theta_eq, X.left.dtheta_eq, ...
    s.X_laststep.left.theta_eq, theta_eq_target_left);

% Body target control is determined by phase and modulated by
% leg force later
u.right.theta_eq = u.right.theta_eq + ...
    eval_pd(p.body_angle_pd, s.phase.right, body_theta_st, X.body.dtheta, 0, 0);
u.left.theta_eq = u.left.theta_eq + ...
    eval_pd(p.body_angle_pd, s.phase.left, body_theta_st, X.body.dtheta, 0, 0);


end


function out = eval_ff(tpoints, phase)

i = 2;
while tpoints(i).phase < phase
    i = i + 1;
end

phase_diff = tpoints(i).phase - tpoints(i - 1).phase;
p = (phase - tpoints(i - 1).phase) / phase_diff;

out = tpoints(i - 1).value + p * (tpoints(i).value - tpoints(i - 1).value);
end


function out = eval_pd(tpoints, phase, x, dx, zero_point, one_point)

i = 2;
while tpoints(i).phase < phase
    i = i + 1;
end

phase_diff = tpoints(i).phase - tpoints(i - 1).phase;
p = (phase - tpoints(i - 1).phase) / phase_diff;

target = tpoints(i - 1).target + p * (tpoints(i).target - tpoints(i - 1).target);
dtarget = (tpoints(i).target - tpoints(i - 1).target) / phase_diff;
kp = tpoints(i - 1).kp + p * (tpoints(i).kp - tpoints(i - 1).kp);
kd = tpoints(i - 1).kd + p * (tpoints(i).kd - tpoints(i - 1).kd);

scale = one_point - zero_point;
offset = zero_point;

out = (kp * ((target * scale + offset) - x)) + (kd * (dtarget * scale - dx));
end


function out = clamp(x, l, h)
out = min(max(x, l), h);
end
