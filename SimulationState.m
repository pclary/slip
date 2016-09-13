function ss = SimulationState(X, cstate, cparams)

if nargin == 0
    X = RobotState();
    cstate = ControllerState();
    cparams = ControllerParams();
end

ss = struct();
ss.X = X;
ss.cstate = cstate;
ss.cparams = cparams;
