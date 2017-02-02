function ground_data = generate_environment_holes()

nholes = 10;

gaps = rand(nholes, 1) + 1;
holes = 0.2*(rand(nholes, 1) + 1);
heights = 0.2*(rand(nholes, 1) - 0.5);

terrain = Terrain();
tdx = (terrain.xend - terrain.xstart) / (numel(terrain.height) - 1);
gaps = tdx * round(gaps / tdx);
holes = tdx * round(holes / tdx);
gx = [gaps, tdx*ones(nholes, 1), holes - tdx, tdx*ones(nholes, 1)]';
gy = [-1e3*ones(nholes, 1), -1e3*ones(nholes, 1), heights, heights]';
ground_x = [-1e3-1; -1e3; cumsum(gx(:)); 1e3; 1e3+1];
ground_y = [-1e3; 0; 0; gy(:); -1e3];


robot = RobotParams();
ground_stiffness = 1e6*ones(size(ground_x));
ground_damping = 1.5*2*sqrt(ground_stiffness*robot.foot.mass);
ground_friction = 1*ones(size(ground_x));
ground_data = [ground_x, ground_y, ground_stiffness, ground_damping, ground_friction];
