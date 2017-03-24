function ground_data = generate_environment_steps()

nsteps = 100;

steps = rand(nsteps, 1) + 1;
heights = cumsum(0.1*sign(rand(nsteps, 1) - 0.5).*(rand(nsteps, 1) + 1));

terrain = Terrain();
tdx = (terrain.xend - terrain.xstart) / (numel(terrain.height) - 1);
steps = tdx * round(steps / tdx);
gx = [steps, tdx*ones(nsteps, 1)]';
gy = [heights, heights]';
ground_x = [-1e4-1; -1e4; cumsum(gx(:)); 1e4; 1e4+1];
ground_y = [-1e4; 0; 0; gy(:); -1e4];


robot = RobotParams();
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*robot.foot.mass);
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];
