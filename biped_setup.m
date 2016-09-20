%% bislip_setup.m

% Robot state
X0 = RobotState();
businfo = Simulink.Bus.createObject(X0);
state_bus = eval(businfo.busName);
clear(businfo.busName);

% Control
u0 = struct();
u0.right.l_eq     = 0;
u0.right.theta_eq = 0;
u0.left.l_eq      = 0;
u0.left.theta_eq  = 0;
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
env = Environment();
businfo = Simulink.Bus.createObject(env);
environment_bus = eval(businfo.busName);
clear(businfo.busName);

% Controller State
cstate0 = ControllerState();
businfo = Simulink.Bus.createObject(cstate0);
cstate_bus = eval(businfo.busName);
clear(businfo.busName);

% Controller Parameters
cparams0 = ControllerParams();
businfo = Simulink.Bus.createObject(cparams0);
cparams_bus = eval(businfo.busName);
clear(businfo.busName);
    
% Stiffness of external body force (dragging body around with mouse)
kp_f_ext = 1e4;
kd_f_ext = 2*sqrt(kp_f_ext*env.body.mass);

% ground_x = [-1e3; -1e3; 5; 5; 10.5; 30; 45; 50; 50; 51; 51; 1e3; 1e3];
% ground_y = [-1e3; 0; 0; -0.3; -0.3; 1; 0; 0; 1; 1; 0; 0; -1e3];
% ground_x = [-1e3; -1e3; 1e3; 1e3];
% ground_y = [-1e3; 0; 0; -1e3];
ground_x = [-1e3; -1e3; 5;  5; 5.5; 5.5; 1e3; 1e3];
ground_y = [-1e3;    0; 0; -1;  -1; 0;   0;  -1e3];
% ground_x = [-1e2; (-1e1:0.1:1e2)'; 1e2];
% ground_y = [-1e2; randn(1101, 1)*0.01; -1e2];
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*env.foot.mass).*ones(size(ground_x));
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];

Ts_controller = 1e-3;
Ts_planner = 1e-2;
Ts_dynamics = 1e-3 / 16;
Ts_visualization = 16e-3;
Ts_sim = 1e-3;
