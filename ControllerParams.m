function p = ControllerParams()

p = struct();
p.phase_rate = 1.5;
p.target_dx = 0;
p.step_offset = 0;
p.energy_injection = 0;
p.step_height = 0.18;
p.max_stride = 0.4;
p.length_pd = struct(...
    'phase',  {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}, ...
    'target', {1.0, 1.0, 1.0, 0.5, 0.0, 0.2, 0.5, 1.0, 1.0, 1.0, 1.0}, ...
    'kp',     {4e3, 4e3, 4e3, 4e3, 4e3, 4e3, 4e3, 4e3, 4e3, 4e3, 4e3}, ...
    'kd',     {1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2});
p.weight_ff = struct(...
    'phase', {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}, ...
    'value', {1.0, 0.1, 1.0, 0.5, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0});
p.energy_ff = struct(...
    'phase', {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}, ...
    'value', {0.0, 0.5, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
p.angle_pd = struct(...
    'phase',  {0.0, 0.4, 0.5, 0.5, 0.6, 1.0}, ...
    'target', {0.0, 0.0, 0.5, 0.0, 1.0, 1.0}, ...
    'kp',     {0.0, 1e3, 1e3, 1e3, 1e3, 0.0}, ...
    'kd',     {0.0, 1e2, 1e2, 1e2, 1e2, 0.0});
p.body_angle_pd = struct(...
    'phase',  { 0.0,  0.15, 0.2,  0.8,  0.85, 1.0}, ...
    'target', { 0.0,  0.0,  0.0,  0.0,  0.0,  0.0}, ...
    'kp',     {-2e3, -2e3,  0.0,  0.0, -2e3, -2e3}, ...
    'kd',     {-1e2, -1e2,  0.0,  0.0, -1e2, -1e2});
