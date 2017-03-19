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
    case 1
        cparams.step_offset = -0.2;
    case 2
        cparams.step_offset = -0.1;
    case 3
        cparams.step_offset = -0.05;
    case 4
        cparams.step_offset = 0;
    case 5
        cparams.step_offset = 0.05;
    case 6
        cparams.step_offset = 0.1;
    case 7
        cparams.step_offset = 0.2;
%     case 1 % Stop
%         cparams.target_dx = 0;
%     case 2 % Standard walk
%         
%     case 3 % Short step
%         cparams.step_offset = -0.1;
%     case 4 % Shorter step
%         cparams.step_offset = -0.2;
%     case 5 % Long step
%         cparams.step_offset = 0.1;
%     case 6 % Longer step
%         cparams.step_offset = 0.2;
%     case 7 % Jump
%         cparams.energy_injection = 400;
%     case 8 % Big jump
%         cparams.energy_injection = 800;
%     case 9 % High step
%         cparams.step_height = cparams.step_height + 0.1;
%     case 10 % Higher step
%         cparams.step_height = cparams.step_height + 0.2;
%     otherwise % Random
%         cparams.step_offset = (rand() - 0.5) * 0.5;
%         cparams.step_height = rand() * 0.2 + 0.1;
%         cparams.energy_injection = (rand() - 0.5) * 1000;
%         cparams.phase_rate = rand() + 1;
%         cparams.target_dx = goal.dx * (rand + 0.5);
end

if gstate.n > 0
    if gstate.n <= 7
        cparams.n = gstate.n;
    else
        cparams.n = 2;
    end
end

gstate.n = gstate.n + 1;

end
