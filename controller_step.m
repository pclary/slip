function [u, c] = controller_step(X, c, p, Ts)

% Get singleturn body angle to allow for flips
X.body.theta = mod(X.body.theta + pi, 2*pi) - pi;

% Construct input layer
input.right = [X.body.theta; X.body.dx; X.body.dy; X.body.dtheta;
    X.right.l_eq; X.right.l - X.right.l_eq; X.right.theta_eq; X.right.theta - X.right.theta_eq;
    X.right.dl_eq; X.right.dl - X.right.dl_eq; X.right.dtheta_eq; X.right.dtheta - X.right.dtheta_eq;
    X.left.l_eq; X.left.l - X.left.l_eq; X.left.theta_eq; X.left.theta - X.left.theta_eq;
    X.left.dl_eq; X.left.dl - X.left.dl_eq; X.left.dtheta_eq; X.left.dtheta - X.left.dtheta_eq;
    sin(2*pi*c.phase.right); cos(2*pi*c.phase.right); sin(2*pi*c.phase.left); cos(2*pi*c.phase.left)];

input.left = [X.body.theta; X.body.dx; X.body.dy; X.body.dtheta;
    X.left.l_eq; X.left.l - X.left.l_eq; X.left.theta_eq; X.left.theta - X.left.theta_eq;
    X.left.dl_eq; X.left.dl - X.left.dl_eq; X.left.dtheta_eq; X.left.dtheta - X.left.dtheta_eq;
    X.right.l_eq; X.right.l - X.right.l_eq; X.right.theta_eq; X.right.theta - X.right.theta_eq;
    X.right.dl_eq; X.right.dl - X.right.dl_eq; X.right.dtheta_eq; X.right.dtheta - X.right.dtheta_eq;
    sin(2*pi*c.phase.left); cos(2*pi*c.phase.left); sin(2*pi*c.phase.right); cos(2*pi*c.phase.right)];


% Hidden layer 1
h1.right = max(p.w1 * [input.right; 1], 0);
h1.left  = max(p.w1 * [input.left; 1], 0);

% Output layer
output.right = p.w2 * [h1.right; 1];
output.left = p.w2 * [h1.left; 1];

% Process pd controller output
output.right(1) = output.right(1) + 0.7;
output.left(1)  = output.left(1) + 0.7;
u = Control();
[u.right.l_eq, c.dfilter_l_right] = ...
    pd(output.right(1:3), X.right.l_eq,     X.right.dl_eq,     c.dfilter_l_right, Ts);
[u.right.theta_eq, c.dfilter_theta_right] = ...
    pd(output.right(4:6), X.right.theta_eq, X.right.dtheta_eq, c.dfilter_theta_right, Ts);
[u.left.l_eq, c.dfilter_l_left] = ...
    pd(output.left(1:3),  X.left.l_eq,      X.left.dl_eq,      c.dfilter_l_left, Ts);
[u.left.theta_eq, c.dfilter_theta_left] = ...
    pd(output.left(4:6),  X.left.theta_eq,  X.left.dtheta_eq,  c.dfilter_theta_left, Ts);

% Update phases
phase_rate = output.right(7) + output.left(7);
c.phase.right = c.phase.right + Ts * phase_rate;
c.phase.left  = c.phase.left  + Ts * phase_rate;

end


function [u, dfilter] = pd(params, x, dx, dfilter, Ts)

% Scale PD parameters
target = params(1);
kp = exp(7 + 2*params(2));
kd = (params(3) + 1) * 2*sqrt(kp);

% Update target derivatives
dtarget_new = (target - dfilter.last) / Ts;
dfilter.dtarget = dfilter.dtarget + 0.3 * (dtarget_new - dfilter.dtarget);
dfilter.last = params(1);

% Compute PD output
u = kp * (target - x) + kd * (dfilter.dtarget - dx);

end
