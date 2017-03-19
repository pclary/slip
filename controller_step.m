function [u, cstate] = controller_step(X, cstate, cparams, Ts)

% Get singleturn body angle to allow for flips
X.body.theta = mod(X.body.theta + pi, 2*pi) - pi;

% Estimate body acceleration
body_ddx_est = (X.body.dx - cstate.body_dx_last) / Ts;
cstate.body_ddx = cstate.body_ddx + cparams.ddx_filter * (body_ddx_est - cstate.body_ddx);
cstate.body_dx_last = X.body.dx;

% Initialize torque struct
u = Control();

% Controller step for each leg
[u.right, cstate.right] = ...
    leg_step(X.body, X.right, cstate.right, cstate.body_ddx, cparams, Ts);
[u.left, cstate.left] = ...
    leg_step(X.body, X.left, cstate.left, cstate.body_ddx, cparams, Ts);

end


function [u, cstate] = leg_step(body, leg, cstate, body_ddx, cparams, Ts)

% Above some speed, increase step rate instead of increasing stride length
if abs(body.dx / cparams.phase_rate) > cparams.max_stride * 2
     cparams.phase_rate = abs(body.dx / cparams.max_stride / 2);
end

dx_diff = clamp(body.dx - cparams.target_dx, -0.5, 0.5);
phase_rate_eq = cparams.phase_rate;
% cparams.phase_rate = cparams.phase_rate + 0.2 * dx_diff * sign(body.dx);

% Add speed-dependent term to energy injection
energy_injection = cparams.energy_injection + 500 * max(abs(body.dx) - 1, 0);

% Energy injection for speed control
energy_injection = energy_injection - dx_diff * 1000 * sign(body.dx);


% Increase leg phases with time
cstate.phase = cstate.phase + Ts * cparams.phase_rate;

% Capture foot positions at phase rollover
if cstate.phase >= 1
    cstate.foot_x_last = body.x + leg.l * sin(body.theta + leg.theta);
end

% Limit phases to [0, 1)
cstate.phase = cstate.phase - floor(cstate.phase);

% Modify timing and trajectory shapes by stretching the phase
phase = stretch_phase(cstate.phase, cparams.phase_stretch);

% Detect ground contact
gc = clamp((leg.l_eq - leg.l) / cparams.contact_threshold, 0, 1);

% Update leg targets
stride_eq = (0.92 / phase_rate_eq^0.3 + 0.1*max(abs(body.dx) - 1, 0) + 0.2*max(abs(body.dx) - 1.8, 0)) * body.dx;
if phase < cparams.step_lock_phase
    foot_extension = (1 - phase) * stride_eq + ...
        0.15 * clamp(body.dx - cparams.target_dx, -0.5, 0.5) + ...
        (0.02 + 0.01 * max(abs(body.dx) - 1, 0)) * body_ddx;
    cstate.foot_x_target = body.x + foot_extension + cparams.step_offset;
end

% Get PD controllers for x and y foot position (leg angle and length)
leg_pd.y = get_pd(cparams.y_pd, phase);
leg_pd.x = get_pd(cparams.x_pd, phase);

% Get transformed leg PD output
u = eval_leg_pd(leg_pd, body, leg, cparams, cstate.foot_x_last, cstate.foot_x_target);

% Modulate angle target control with ground contact
u.theta_eq = (1 - gc) * u.theta_eq;

% Add feedforward terms for weight compensation and energy injection
u.l_eq = u.l_eq + ...
    eval_ff(cparams.weight_ff, phase) * cparams.robot_weight + ...
    eval_ff(cparams.energy_ff, phase) * energy_injection;

% Add body angle control, modulated with ground contact
u.theta_eq = u.theta_eq - ...
    gc * eval_pd(cparams.body_angle_pd, phase, body.theta, body.dtheta, 0, 0);

end


function out = eval_ff(tpoints, phase)

i = 2 + floor(phase * 10);
p = phase * 10 - i + 2;

out = tpoints(i - 1) + p * (tpoints(i) - tpoints(i - 1));
end


function out = eval_pd(tpoints, phase, x, dx, zero_point, one_point)

i = 2 + floor(phase * 10);
phase_diff = 0.1;
p = phase * 10 - i + 2;

target = tpoints(i - 1).target + p * (tpoints(i).target - tpoints(i - 1).target);
dtarget = (tpoints(i).target - tpoints(i - 1).target) / phase_diff;
kp = tpoints(i - 1).kp + p * (tpoints(i).kp - tpoints(i - 1).kp);
kd = tpoints(i - 1).kd + p * (tpoints(i).kd - tpoints(i - 1).kd);

scale = one_point - zero_point;
offset = zero_point;

out = (kp * ((target * scale + offset) - x)) + (kd * (dtarget * scale - dx));
end


function pd = get_pd(tpoints, phase)

i = 2 + floor(phase * 10);
phase_diff = 0.1;
p = phase * 10 - i + 2;

pd.target = tpoints(i - 1).target + p * (tpoints(i).target - tpoints(i - 1).target);
pd.dtarget = (tpoints(i).target - tpoints(i - 1).target) / phase_diff;
pd.kp = tpoints(i - 1).kp + p * (tpoints(i).kp - tpoints(i - 1).kp);
pd.kd = tpoints(i - 1).kd + p * (tpoints(i).kd - tpoints(i - 1).kd);
end


function u = eval_leg_pd(pd, body, leg, p, x_last, x_target)
% Get scaled x and y targets
x = pd.x.target * (x_target - x_last) + x_last - body.x;
dx = pd.x.dtarget * (x_target - x_last) - body.dx;
y = p.l_max - pd.y.target * p.step_height;
dy = -pd.y.dtarget * p.step_height;

% Transform to polar
l = sqrt(x^2 + y^2);
dl = (x*dx + y*dy) / l;
theta = real(asin(complex(x / l))) - body.theta;
dtheta = (y*dx - x*dy) / l^2 - body.dtheta;

% Compute PD controllers
u.l_eq = pd.y.kp * (l - leg.l_eq) + pd.y.kd * (dl - leg.dl_eq);
u.theta_eq = pd.x.kp * (theta - leg.theta_eq) + pd.x.kd * (dtheta - leg.dtheta_eq);
end


function p = stretch_phase(p, stretch)
% Stretch the phase with a sigmoid to lengthen or shorten the middle relative
% to the ends

if abs(stretch) < 1e4*eps
    return
end

b = 1/(2*(exp(stretch/2) - 1));
if p < 0.5
    p = b*exp(stretch*p) - b;
else
    p = 1 - (b*exp(stretch*(1 - p)) - b);
end
end


function out = clamp(x, l, h)
out = min(max(x, l), h);
end
