function [u, c] = controller_step(X, c, p, Ts)

% Get singleturn body angle to allow for flips
X.body.theta = mod(X.body.theta + pi, 2*pi) - pi;

% Construct input layer
input = [X.body.theta; X.body.dx; X.body.dy; X.body.dtheta;
    X.right.l_eq; X.right.l - X.right.l_eq; X.right.theta_eq; X.right.theta - X.right.theta_eq;
    X.right.dl_eq; X.right.dl - X.right.dl_eq; X.right.dtheta_eq; X.right.dtheta - X.right.dtheta_eq;
    X.left.l_eq; X.left.l - X.left.l_eq; X.left.theta_eq; X.left.theta - X.left.theta_eq;
    X.left.dl_eq; X.left.dl - X.left.dl_eq; X.left.dtheta_eq; X.left.dtheta - X.left.dtheta_eq;
    sin(2*pi*c.phase.right); cos(2*pi*c.phase.right); sin(2*pi*c.phase.left); cos(2*pi*c.phase.left)];

% Hidden layer 1
h1 = max(p.w1 * [input; 1], 0);

% Output layer
output = p.w2 * [h1; 1];

% Process output
u = Control();
u.right.l_eq     = 12*output(1);
u.right.theta_eq = 12*output(2);
u.left.l_eq      = 12*output(3);
u.left.theta_eq  = 12*output(4);

c.phase.right = c.phase.right + Ts * output(5);
c.phase.left  = c.phase.left  + Ts * output(5);

end
