%% bislip_setup.m

% Robot state
X0 = struct();
X0.body.x      = 0;
X0.body.y      = 1;
X0.body.theta  = 0;
X0.body.dx     = 0;
X0.body.dy     = 0;
X0.body.dtheta = 0;
X0.right.l         = 1;
X0.right.l_eq      = 1;
X0.right.theta     = 0;
X0.right.theta_eq  = 0;
X0.right.dl        = 0;
X0.right.dl_eq     = 0;
X0.right.dtheta    = 0;
X0.right.dtheta_eq = 0;
X0.left.l         = 1;
X0.left.l_eq      = 1;
X0.left.theta     = 0;
X0.left.theta_eq  = 0;
X0.left.dl        = 0;
X0.left.dl_eq     = 0;
X0.left.dtheta    = 0;
X0.left.dtheta_eq = 0;
businfo = Simulink.Bus.createObject(X0);
state_bus = eval(businfo.busName);
clear(businfo.busName);

% Control
u0 = struct();
u0.right.l_eq.torque  = 0;
u0.right.l_eq.target  = 0;
u0.right.l_eq.dtarget = 0;
u0.right.l_eq.kp      = 0;
u0.right.l_eq.kd      = 0;
u0.right.theta_eq.torque  = 0;
u0.right.theta_eq.target  = 0;
u0.right.theta_eq.dtarget = 0;
u0.right.theta_eq.kp      = 0;
u0.right.theta_eq.kd      = 0;
u0.left.l_eq.torque  = 0;
u0.left.l_eq.target  = 0;
u0.left.l_eq.dtarget = 0;
u0.left.l_eq.kp      = 0;
u0.left.l_eq.kd      = 0;
u0.left.theta_eq.torque  = 0;
u0.left.theta_eq.target  = 0;
u0.left.theta_eq.dtarget = 0;
u0.left.theta_eq.kp      = 0;
u0.left.theta_eq.kd      = 0;
businfo = Simulink.Bus.createObject(u0);
control_bus = eval(businfo.busName);
clear(businfo.busName);

% External forces
ext0 = struct();
ext0.body.x     = 0;
ext0.body.y     = 0;
ext0.body.theta = 0;
ext0.right.x    = 0;
ext0.right.y    = 0;
ext0.left.x     = 0;
ext0.left.y     = 0;
businfo = Simulink.Bus.createObject(ext0);
external_force_bus = eval(businfo.busName);
clear(businfo.busName);

% Environment
env = struct();
env.gravity               = 9.81;
env.body.mass             = 30;
env.body.inertia          = 0.3;
env.foot.mass             = 0.4;
env.length.stiffness      = 1e4;
env.length.damping        = 1e2;
env.length.motor.inertia  = 1e-3;
env.length.motor.damping  = 1e-1;
env.length.motor.ratio    = 48;
env.length.motor.torque   = 12;
env.length.hardstop.min   = 0.3;
env.length.hardstop.max   = 1;
env.length.hardstop.kp    = 4e3;
env.length.hardstop.kd    = 4e1;
env.length.hardstop.dfade = 1e-2;
env.length.hardstop.fmax  = 1e5;
env.angle.stiffness       = 1e4;
env.angle.damping         = 1e2;
env.angle.motor.inertia   = 1e-3;
env.angle.motor.damping   = 1e-1;
env.angle.motor.ratio     = 16;
env.angle.motor.torque    = 12;
env.angle.hardstop.min    = -1.5;
env.angle.hardstop.max    = 1.5;
env.angle.hardstop.kp     = 1e3;
env.angle.hardstop.kd     = 1e1;
env.angle.hardstop.dfade  = 1e-2;
env.angle.hardstop.fmax   = 1e3;
env.ground.damping_depth  = 1e-3;
env.ground.slip_ramp      = 1e-4;
businfo = Simulink.Bus.createObject(env);
environment_bus = eval(businfo.busName);
clear(businfo.busName);
    
% Stiffness of external body force (dragging body around with mouse)
kp_f_ext = 1e4;
kd_f_ext = 2*sqrt(kp_f_ext*env.body.mass);

%ground_x = [-1e3; -1e3; 5; 5; 10.5; 30; 45; 50; 50; 51; 51; 1e3; 1e3];
%ground_y = [-1e3; 0; 0; -0.3; -0.3; 1; 0; 0; 1; 1; 0; 0; -1e3];
ground_x = [-1e3; -1e3; 1e3; 1e3];
ground_y = [-1e3; 0; 0; -1e3];
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*env.foot.mass).*ones(size(ground_x));
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];

Ts = 1e-3;

