function terrain = Terrain()

terrain.xstart = -2;
terrain.xend = 2;
terrain.height = zeros(201, 1);

terrain.stiffness = 1e6;
terrain.damping = 2e3;
terrain.friction = 1;
