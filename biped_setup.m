%% bislip_setup.m

% Robot state
X0 = RobotState();
businfo = Simulink.Bus.createObject(X0);
state_bus = eval(businfo.busName);
clear(businfo.busName);

% Control
u0 = Control();
businfo = Simulink.Bus.createObject(u0);
control_bus = eval(businfo.busName);
clear(businfo.busName);

% External forces
ext0 = ExternalForces();
businfo = Simulink.Bus.createObject(ext0);
external_force_bus = eval(businfo.busName);
clear(businfo.busName);

% Environment
robot = RobotParams();
businfo = Simulink.Bus.createObject(robot);
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
kd_f_ext = 2*sqrt(kp_f_ext*robot.body.mass);

ground_x = [-1e3; -1e3+1; 5; 5.01; 10.5; 30; 45; 50; 50.01; 51; 51.01; 1e3-1; 1e3];
ground_y = [-1e3; 0; 0; -0.5; -0.5; 1; 0; 0; 1; 1; 0; 0; -1e3];
% ground_x = [-1e3; -1e3; 1e3; 1e3];
% ground_y = [-1e3; 0; 0; -1e3];
% ground_x = [-1e3; -1e3; 3;  3; 3.5; 3.5; 4.8; 4.8; 5.1; 5.1; 8; 8; 8.6; 8.6; 1e3; 1e3];
% ground_y = [-1e3;    0; 0; -1;  -1; 0; 0; -1;  -1; 0; 0; -1;  -1; 0;   0;  -1e3];
% ground_x = [-2e1; (-2e1:0.1:2e1)'; 2e1];
% ground_y = [-2e1; randn(401, 1)*0.01; -2e1];
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*robot.foot.mass);
ground_friction = 1*ones(size(ground_x));
env = Environment();
env.ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];

Ts_controller = 1e-3;
Ts_planner = 1e-3 * 2;
Ts_dynamics = 1e-3 / 8;
Ts_visualization = 16e-3;
Ts_sim = 1e-3;
