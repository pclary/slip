function [cparams, gstate] = generate_params(X, goal, ground_data, gstate)

cparams = ControllerParams();
cparams.target_dx = goal.dx;

if gstate.n == 1
    cparams.target_dx = 0;
end

if gstate.n > 1
    cparams.step_offset = (rand() - 0.5) * 0.5;
end
    
if gstate.n > 4
    cparams.step_height = rand() * 0.2 + 0.1;
end
    
if gstate.n > 7
    cparams.energy_injection = (rand() - 0.5) * 1000;
end

if gstate.n > 10
    cparams.phase_rate = rand() + 1;
end

if gstate.n > 13
    cparams.target_dx = goal.dx * (rand + 0.5);
end

gstate.n = gstate.n + 1;

end
