function [cparams, gstate] = generate_params(X, goal, ground_data, gstate)

cparams = ControllerParams();
cparams.target_dx = goal.dx;

if gstate.n > 0
    cparams.step_offset = (rand() - 0.5) * 0.5;
end
    
if gstate.n > 3
    cparams.energy_injection = (rand() - 0.5) * 1000;
end

if gstate.n > 6
    cparams.phase_rate = rand() + 1;
end

if gstate.n > 9
    cparams.target_dx = goal.dx * (rand + 0.5);
end

gstate.n = gstate.n + 1;

end
