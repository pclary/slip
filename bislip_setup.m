%% bislip_setup.m

l0 = 1;
h0 = 1;
tha0 = 0.0;
thb0 = -0.0;
v0 = 0;%1;
X0 = [0; l0+h0; v0; 0; 0; 0;
      l0*sin(tha0); l0*(1-cos(tha0)) + h0; v0; 0; l0;
      l0*sin(thb0); l0*(1-cos(thb0)) + h0; v0; 0; l0];
u0 = [0; 0; 0; 0];

body_mass = 50;
body_inertia = 5;
foot_mass = 25;
leg_stiffness = 5e3;
leg_damping = 50;
gravity = 0;%9.81;

ground_x = [-1e3; 1e3];
ground_y = [0; 0];
ground_stiffness = 1e5*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*foot_mass).*ones(size(ground_x));
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];

Ts = 1e-3;
