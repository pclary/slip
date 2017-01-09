%% Sets up workspace for simulation

% Delete all existing bus objects
vars = who();
for i = 1:numel(vars)
    if isa(eval(vars{i}), 'Simulink.Bus')
        clear(vars{i})
    end
end
clear i vars

% Create bus objects
biped_bus_definitions();

% Set robot parameters
robot = RobotParams();

% Initial robot state
X0 = RobotState();
X0.body.dx = 1;

% Initial controller state
cstate0 = ControllerState();
    
% Stiffness of external body force (dragging body around with mouse)
kp_f_ext = 1e4;
kd_f_ext = 2*sqrt(kp_f_ext*robot.body.mass);

% Ground information
% ground_x = [-1e3; -1e3+1; 5; 5.01; 10.5; 30; 45; 50; 50.01; 51; 51.01; 1e3-1; 1e3];
% ground_y = [-1e3; 0; 0; -0.5; -0.5; 1; 0; 0; 1; 1; 0; 0; -1e3];
% ground_x = [-1e3-1; -1e3; 1e3; 1e3+1];
% ground_y = [-1e3; 0; 0; -1e3];
ground_x = [-1e3-1; -1e3; 3;  3.01; 3.5; 3.51; 4.8; 4.81; 5.1; 5.11; 8; 8.01; 8.6; 8.61; 1e3; 1e3+1];
ground_y = [-1e3;    0; 0; -1e3;  -1e3; 0; 0; -1e3;  -1e3; 0; 0; -1e3;  -1e3; 0;   0;  -1e3];
% ground_x = [-2e1; (-2e1:0.1:2e1)'; 2e1];
% ground_y = [-2e1; randn(401, 1)*0.01; -2e1];
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*robot.foot.mass);
ground_friction = 1*ones(size(ground_x));
env = Environment([ground_x, ground_y, ground_stiffness, ground_damping, ground_friction]);
clear ground_x ground_y ground_stiffness ground_damping ground_friction

% Timesteps
Ts_controller = 1e-3;
Ts_planner = 1e-3 * 2;
Ts_dynamics = 1e-3 / 8;
Ts_visualization = 16e-3;
Ts_sim = 1e-3;
