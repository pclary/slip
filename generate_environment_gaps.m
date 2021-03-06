function ground_data = generate_environment_gaps()

nholes = 100;

gaps = rand(nholes, 1) + 1;
holes = 0.15*(rand(nholes, 1) + 1);
heights = 0*(0.1*(rand(nholes, 1) - 0.5) - holes / 2);

terrain = Terrain();
tdx = (terrain.xend - terrain.xstart) / (numel(terrain.height) - 1);
gaps = tdx * round(gaps / tdx);
holes = tdx * round(holes / tdx);
gx = [gaps, tdx*ones(nholes, 1), holes - tdx, tdx*ones(nholes, 1)]';
gy = [-1e4*ones(nholes, 1), -1e4*ones(nholes, 1), heights, heights]';
ground_x = [-1e4-1; -1e4; cumsum(gx(:)); 1e4; 1e4+1];
ground_y = [-1e4; 0; 0; gy(:); -1e4];


robot = RobotParams();
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*robot.foot.mass);
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];
