function ss = SimulationState(X, cstate, cparams, gstate, value, rollout_value)

if nargin == 0
    X       = RobotState();
    cstate  = ControllerState();
    cparams = ControllerParams();
    gstate  = GeneratorState();
    value   = 0;
    rollout_value = 0;
end

ss.X       = X;
ss.cstate  = cstate;
ss.cparams = cparams;
ss.gstate  = gstate;
ss.value   = value;
ss.rollout_value = rollout_value;
