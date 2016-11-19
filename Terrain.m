function terrain = Terrain()

terrain.xstart = -1;
terrain.xend = 1;
terrain.height = zeros(101, 1);

terrain.stiffness = 1e6;
terrain.damping = 2e3;
terrain.friction = 1;
