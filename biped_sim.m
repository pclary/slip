function [X, cstate, X_delay, u] = biped_sim(X, cstate, robot, cparams, terrain, tstop, Ts, noise)

noiseval = zeros(4, 1);

% Dynamics steps per controller step
ratio = 8;

% X value delayed by 1 controller cycle
X_delay = X;
u = controller_step(X_delay, cstate, cparams, Ts * ratio);

% No external forces
ext0 = ExternalForces();

t = 0;

% Fixed-step RK4 integration
while t < tstop
    
    for i = 1:ratio
        X1 = X;
        dX1 = biped_dynamics(X1, u, ext0, robot, terrain);
        
        X2 = rs_add(X1, rs_smul(dX1, Ts/2));
        dX2 = biped_dynamics(X2, u, ext0, robot, terrain);
        
        X3 = rs_add(X1, rs_smul(dX2, Ts/2));
        dX3 = biped_dynamics(X3, u, ext0, robot, terrain);
        
        X4 = rs_add(X1, rs_smul(dX3, Ts));
        dX4 = biped_dynamics(X4, u, ext0, robot, terrain);
        
        X = rs_add(X1, rs_smul(rs_add(rs_add(dX1, rs_smul(dX2, 2)), rs_add(rs_smul(dX3, 2), dX4)), Ts/6));
        t = t + Ts;
    end
    
    [u, cstate] = controller_step(X_delay, cstate, cparams, Ts * ratio);
    
    f = 1e-2;
    noiseval = (1 - f) * noiseval + f * noise * randn(4, 1);
    
    u.right.l_eq     = u.right.l_eq + noiseval(1);
    u.right.theta_eq = u.right.theta_eq + noiseval(2);
    u.left.l_eq      = u.left.l_eq + noiseval(3);
    u.left.theta_eq  = u.left.theta_eq + noiseval(4);
    
    X_delay = X;
    
    % Stop if crashed
    if X.body.y < min(terrain.height)
        break
    end
end

end


function c = rs_add(a, b)
    c.body.x = a.body.x + b.body.x;
    c.body.y = a.body.y + b.body.y;
    c.body.theta = a.body.theta + b.body.theta;
    c.body.dx = a.body.dx + b.body.dx;
    c.body.dy = a.body.dy + b.body.dy;
    c.body.dtheta = a.body.dtheta + b.body.dtheta;
    
    c.right.l = a.right.l + b.right.l;
    c.right.l_eq = a.right.l_eq + b.right.l_eq;
    c.right.theta = a.right.theta + b.right.theta;
    c.right.theta_eq = a.right.theta_eq + b.right.theta_eq;
    c.right.dl = a.right.dl + b.right.dl;
    c.right.dl_eq = a.right.dl_eq + b.right.dl_eq;
    c.right.dtheta = a.right.dtheta + b.right.dtheta;
    c.right.dtheta_eq = a.right.dtheta_eq + b.right.dtheta_eq;
    
    c.left.l = a.left.l + b.left.l;
    c.left.l_eq = a.left.l_eq + b.left.l_eq;
    c.left.theta = a.left.theta + b.left.theta;
    c.left.theta_eq = a.left.theta_eq + b.left.theta_eq;
    c.left.dl = a.left.dl + b.left.dl;
    c.left.dl_eq = a.left.dl_eq + b.left.dl_eq;
    c.left.dtheta = a.left.dtheta + b.left.dtheta;
    c.left.dtheta_eq = a.left.dtheta_eq + b.left.dtheta_eq;
end


function c = rs_smul(a, b)
    c.body.x = a.body.x * b;
    c.body.y = a.body.y * b;
    c.body.theta = a.body.theta * b;
    c.body.dx = a.body.dx * b;
    c.body.dy = a.body.dy * b;
    c.body.dtheta = a.body.dtheta * b;
    
    c.right.l = a.right.l * b;
    c.right.l_eq = a.right.l_eq * b;
    c.right.theta = a.right.theta * b;
    c.right.theta_eq = a.right.theta_eq * b;
    c.right.dl = a.right.dl * b;
    c.right.dl_eq = a.right.dl_eq * b;
    c.right.dtheta = a.right.dtheta * b;
    c.right.dtheta_eq = a.right.dtheta_eq * b;
    
    c.left.l = a.left.l * b;
    c.left.l_eq = a.left.l_eq * b;
    c.left.theta = a.left.theta * b;
    c.left.theta_eq = a.left.theta_eq * b;
    c.left.dl = a.left.dl * b;
    c.left.dl_eq = a.left.dl_eq * b;
    c.left.dtheta = a.left.dtheta * b;
    c.left.dtheta_eq = a.left.dtheta_eq * b;
end


