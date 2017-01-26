function [cparams, gstate] = generate_params(X, goal, terrain, gstate, action_queue)

if ~action_queue.isempty()
    cparams = action_queue.pop();
    return;
end

cparams = ControllerParams();
cparams.target_dx = goal.dx;

switch gstate.n
    case 0 % Same as last step
        cparams = gstate.last_cparams;
    case 1 % Stop
        cparams.target_dx = 0;
    case 2 % Short step
        cparams.step_offset = -0.1;
    case 3 % Shorter step
        cparams.step_offset = -0.2;
    case 4 % Long step
        cparams.step_offset = 0.1;
    case 5 % Longer step
        cparams.step_offset = 0.2;
    case 6 % Jump
        cparams.energy_injection = 400;
    case 7 % Big jump
        cparams.energy_injection = 800;
    case 8 % High step
        cparams.step_height = cparams.step_height + 0.1;
    case 9 % Higher step
        cparams.step_height = cparams.step_height + 0.2;
    otherwise % Random
        cparams.step_offset = (rand() - 0.5) * 0.5;
        cparams.step_height = rand() * 0.2 + 0.1;
        cparams.energy_injection = (rand() - 0.5) * 1000;
        cparams.phase_rate = rand() + 1;
        cparams.target_dx = goal.dx * (rand + 0.5);
end

gstate.n = gstate.n + 1;

end
