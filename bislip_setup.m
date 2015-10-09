%% bislip_setup.m

l0 = 1;
h0 = 0.2;
th0 = 0.5;
v0 = 0.2;
Y0 = [0; l0+h0; 0; v0; 0; 0;
    l0*sin(th0); l0*(1-cos(th0)) + h0; 0; 0;
    -l0*sin(th0); l0*(1-cos(th0)) + h0; 0; 0];

body_mass = 10;
body_inertia = 1;
foot_mass = 1;
leg_stiffness = 1e3;
leg_damping = 10;
gravity = 9.81;

ground_x = [-1e3; 1e3];
ground_y = [0; 0];
ground_stiffness = 1e5*ones(size(ground_x));
ground_damping = 10*ones(size(ground_x));
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];
