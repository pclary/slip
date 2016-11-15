function [u, s] = controller_step(X, s, p, Ts)

% Get singleturn body angle to allow for flips
X.body.theta = mod(X.body.theta + pi, 2*pi) - pi;

% Above some speed, increase step rate instead of increasing stride length
if abs(X.body.dx / p.phase_rate) > p.max_stride * 2
     p.phase_rate = abs(X.body.dx / p.max_stride / 2);
end

% Increase leg phases with time
s.phase.right = s.phase.right + Ts * p.phase_rate;
s.phase.left  = s.phase.left  + Ts * p.phase_rate;

% Capture foot positions at phase rollover
if s.phase.right >= 1
    s.foot_x_last.right = X.body.x + X.right.l * sin(X.body.theta + X.right.theta);
end
if s.phase.left >= 1
    s.foot_x_last.left = X.body.x + X.left.l * sin(X.body.theta + X.left.theta);
end

% Limit phases to [0, 1)
s.phase.right = s.phase.right - floor(s.phase.right);
s.phase.left  = s.phase.left  - floor(s.phase.left);

% Estimate body acceleration
body_ddx_est = (X.body.dx - s.body_dx_last) / Ts;
s.body_ddx = s.body_ddx + p.ddx_filter * (body_ddx_est - s.body_ddx);
s.body_dx_last = X.body.dx;

% Update leg targets
foot_extension = X.body.dx / (2 * p.phase_rate) + ...
    0.1 * clamp(X.body.dx - p.target_dx, -0.5, 0.5) + ...
    0.05 * s.body_ddx;
if s.phase.right < p.step_lock_phase
    s.foot_x_target.right = X.body.x + foot_extension;
end
if s.phase.left < p.step_lock_phase
    s.foot_x_target.left = X.body.x + foot_extension;
end

% Initialize torque struct
u.right.l_eq = 0;
u.right.theta_eq = 0;
u.left.l_eq = 0;
u.left.theta_eq = 0;

% Get PD controllers for x and y foot position (leg angle and length)
leg_pd.right.y = get_pd(p.y_pd, s.phase.right);
leg_pd.right.x = get_pd(p.x_pd, s.phase.right);
leg_pd.left.y  = get_pd(p.y_pd, s.phase.left);
leg_pd.left.x  = get_pd(p.x_pd, s.phase.left);

% Get transformed leg PD output
u.right = eval_leg_pd(leg_pd.right, X, X.right, p, s.foot_x_last.right, s.foot_x_target.right);
u.left  = eval_leg_pd(leg_pd.left,  X, X.left,  p, s.foot_x_last.left,  s.foot_x_target.left);

% Add feedforward terms for weight compensation and energy injection
u.right.l_eq = u.right.l_eq + ...
    eval_ff(p.weight_ff, s.phase.right) * p.robot_weight + ...
    eval_ff(p.energy_ff, s.phase.right) * p.energy_injection;
u.left.l_eq = u.left.l_eq + ...
    eval_ff(p.weight_ff, s.phase.left) * p.robot_weight + ...
    eval_ff(p.energy_ff, s.phase.left) * p.energy_injection;

% Add body angle control, modulated with ground contact
gc.right = 1;%clamp((X.right.l_eq - X.right.l) / p.contact_threshold, 0, 1);
gc.left = 1;%clamp((X.left.l_eq - X.left.l) / p.contact_threshold, 0, 1);
u.right.theta_eq = u.right.theta_eq - ...
    gc.right * eval_pd(p.body_angle_pd, s.phase.right, X.body.theta, X.body.dtheta, 0, 0);
u.left.theta_eq = u.left.theta_eq - ...
    gc.left * eval_pd(p.body_angle_pd, s.phase.left, X.body.theta, X.body.dtheta, 0, 0);

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


function pd = get_pd(tpoints, phase)

i = 2;
while tpoints(i).phase < phase
    i = i + 1;
end

phase_diff = tpoints(i).phase - tpoints(i - 1).phase;
p = (phase - tpoints(i - 1).phase) / phase_diff;

pd.target = tpoints(i - 1).target + p * (tpoints(i).target - tpoints(i - 1).target);
pd.dtarget = (tpoints(i).target - tpoints(i - 1).target) / phase_diff;
pd.kp = tpoints(i - 1).kp + p * (tpoints(i).kp - tpoints(i - 1).kp);
pd.kd = tpoints(i - 1).kd + p * (tpoints(i).kd - tpoints(i - 1).kd);
end


function u = eval_leg_pd(pd, X, leg, p, x_last, x_target)
% Get scaled x and y targets
x = pd.x.target * (x_target - x_last) + x_last - X.body.x;
dx = pd.x.dtarget * (x_target - x_last) - X.body.dx;
y = p.l_max - pd.y.target * p.step_height;
dy = -pd.y.dtarget * p.step_height;

% Transform to polar
l = sqrt(x^2 + y^2);
dl = (x*dx + y*dy) / l;
theta = real(asin(complex(x / l))) - X.body.theta;
dtheta = (y*dx - x*dy) / l^2 - X.body.dtheta;

% Compute PD controllers
u.l_eq = pd.y.kp * (l - leg.l_eq) + pd.y.kd * (dl - leg.dl_eq);
u.theta_eq = pd.x.kp * (theta - leg.theta_eq) + pd.x.kd * (dtheta - leg.dtheta_eq);
end


function out = clamp(x, l, h)
out = min(max(x, l), h);
end
