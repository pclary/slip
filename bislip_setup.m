%% bislip_setup.m

l0 = 1;
h0 = 0.1;
tha0 = 0.3;
thb0 = -0.3;
v0 = 0.5;
X0 = [0; v0; l0+h0; 0; 0;    0;
     l0; 0;  l0;    0; tha0; 0;
     l0; 0;  l0;    0; thb0; 0];
u0 = [0; 0; 0; 0];

body_mass = 75;
body_inertia = 10;
foot_mass = 1;
leg_stiffness = 1.1e4;
leg_damping = 0.03*2*sqrt(leg_stiffness*body_mass);
length_motor_inertia = 1;
length_motor_damping = 0.1;
angle_motor_inertia = 0.1;
angle_motor_damping = 0.05;
angle_motor_ratio = 16;
gravity = 9.81;
params = [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping; 
          length_motor_inertia; length_motor_damping; angle_motor_inertia; 
          angle_motor_damping; angle_motor_ratio; gravity];

ground_x = [-1e3; 1e3];
ground_y = [0; 0];
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*foot_mass).*ones(size(ground_x));
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];

Ts = 1e-3;
