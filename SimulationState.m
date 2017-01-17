function ss = SimulationState(X, cstate, cparams, gstate, stability, goal_value, path_value)

if nargin == 0
    X       = RobotState();
    cstate  = ControllerState();
    cparams = ControllerParams();
    gstate  = GeneratorState();
    stability  = 0;
    goal_value = 0;
    path_value = 0;
end

ss.X       = X;
ss.cstate  = cstate;
ss.cparams = cparams;
ss.gstate  = gstate;
ss.stability  = stability;
ss.goal_value = goal_value;
ss.path_value = path_value;
