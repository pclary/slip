function terrain = Terrain()

terrain.xstart = -4;
terrain.xend = 4;
terrain.height = zeros(201, 1);

terrain.stiffness = 1e6;
terrain.damping = 2e3;
terrain.friction = 1;
