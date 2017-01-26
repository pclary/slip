function ss = SimulationState(varargin)

nsamples = 1;
if nargin == 1
    nsamples = varargin{1};
end

if nargin <= 1
    ss.X       = repmat(RobotState(), nsamples, 1);
    ss.cstate  = repmat(ControllerState(), nsamples, 1);
    ss.cparams = ControllerParams();
    ss.gstate  = GeneratorState();
    ss.stability  = 0;
    ss.goal_value = 0;
    ss.path_value = 0;
else
    ss.X       = varargin{1};
    ss.cstate  = varargin{2};
    ss.cparams = varargin{3};
    ss.gstate  = varargin{4};
    ss.stability  = varargin{5};
    ss.goal_value = varargin{6};
    ss.path_value = varargin{7};
end
