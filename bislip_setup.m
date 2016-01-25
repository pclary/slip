%% bislip_setup.m

l0 = 1;
h0 = 0.0;
tha0 = 0.1;
thb0 = -0.1;
v0 = 0;
X0 = [0; v0; l0+h0; 0; 0;    0;
     l0; 0;  l0;    0; tha0; 0;
     l0; 0;  l0;    0; thb0; 0];
u0 = [0; 0; 0; 0];

body_mass = 50;
body_inertia = 2; % 20cm radius of gyration
foot_mass = 0.5;
leg_stiffness = 1e4; % 5cm deflection caused by gravity (one leg)
leg_damping = 0.01*2*sqrt(leg_stiffness*body_mass); % damping ratio of 0.01 (~94% efficiency)
length_motor_inertia = 2e-3*16^2; % 2e-3 rotational inertia, 16:1 gearbox
length_motor_damping = 20; % made it bigger than 4
angle_motor_inertia = 2e-3; % 1 kg with ~4cm radius of gyration
angle_motor_damping = 0.08; % linked to length motor damping
angle_motor_ratio = 16;
gravity = 9.81;
params = [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping; 
          length_motor_inertia; length_motor_damping; angle_motor_inertia; 
          angle_motor_damping; angle_motor_ratio; gravity];

% [length, angle, body_angle]
kp_ground = [6e4; 0; 5e1];
kd_ground = [2e2; 0; 4];
kp_air = [6e3; 1e2; 0];
kd_air = [2e2; 20; 0];
    
% Stiffness of external body force (dragging body around with mouse)
kp_f_ext = 1e4;
kd_f_ext = 2*sqrt(kp_f_ext*body_mass);

%ground_x = [-1e3; -1e3; 5; 5; 10.5; 30; 45; 50; 50; 51; 51; 1e3; 1e3];
%ground_y = [-1e3; 0; 0; -0.3; -0.3; 1; 0; 0; 1; 1; 0; 0; -1e3];
ground_x = [-1e3; -1e3; 1e3; 1e3];
ground_y = [-1e3; 0; 0; -1e3];
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*foot_mass).*ones(size(ground_x));
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];

Ts = 1e-3;

% Define sequencing signal bus type
sequencing_signal_struct.forces = [0; 0];
sequencing_signal_struct.forces_filtered = [0; 0];
sequencing_signal_struct.feet_fade = [0; 0];
sequencing_signal_struct.feet_contact_good = [false; false];
sequencing_signal_struct.touchdown = [false; false];
sequencing_signal_struct.touchdown_fast = [false; false];
sequencing_signal_struct.midstance = [false; false];
sequencing_signal_struct.takeoff = [false; false];
sequencing_signal_struct.takeoff_fast = [false; false];
sequencing_signal_struct.phase = [0; 0];
businfo = Simulink.Bus.createObject(sequencing_signal_struct);
sequencing_signal_bus = eval(businfo.busName);
clear(businfo.busName);
